"""Natural-language front end for the Go2: a person's text prompt -> an LLM that can
call the go2_mcp_server tools (list_places, navigate_to_place, get_navigation_status,
cancel_navigation) -> Nav2 actually drives the robot.

The LLM never sees/produces coordinates and never touches /cmd_vel -- it can only choose
a place name from list_places() and call navigate_to_place(name). Nav2 remains in charge
of the actual navigation.

Talks to any OpenAI-compatible chat-completions endpoint (tools/function-calling
included), configured entirely through env vars -- no code change needed to switch
providers:
    GO2_LLM_BASE_URL  -- default: Gemini's OpenAI-compatible endpoint
    GO2_LLM_API_KEY   -- required for Gemini; get a free one at
                         https://aistudio.google.com/app/apikey (falls back to the
                         GEMINI_API_KEY env var if that's already set)
    GO2_LLM_MODEL     -- default: gemini-3.1-flash-lite

To go back to a local/offline model instead (e.g. Ollama), set GO2_LLM_BASE_URL to
Ollama's own OpenAI-compatible endpoint (http://127.0.0.1:11434/v1) and leave
GO2_LLM_API_KEY unset -- no code changes needed either way.

Usage:
    ros2 run go2_llm_nav go2_llm_nav                  # interactive REPL
    ros2 run go2_llm_nav go2_llm_nav "go to the sofa"  # single prompt, then exit
"""

import asyncio
import json
import os
import sys

import httpx
from ament_index_python.packages import get_package_prefix
from mcp import ClientSession, StdioServerParameters, stdio_client

LLM_BASE_URL = os.environ.get(
    "GO2_LLM_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai"
).rstrip("/")
LLM_API_KEY = os.environ.get("GO2_LLM_API_KEY") or os.environ.get("GEMINI_API_KEY", "")
LLM_MODEL = os.environ.get("GO2_LLM_MODEL", "gemini-3.1-flash-lite")

SYSTEM_PROMPT = """You are the natural-language interface for a Unitree Go2 mobile robot.

You control robot navigation ONLY through the tools you are given. You must NEVER invent
a *named place*, and you have no way to move the robot except through these tools. If
you are unsure what places exist, call list_places first. The exception to "never invent
coordinates" is rotate(angle_deg) and navigate_to_point(x, y) -- these are raw movement
primitives, always available, and picking your own angle/coordinates with them is the
whole point (see MOVING FREELY / LOOKING AROUND below); Nav2 handles the actual path
planning and obstacle avoidance underneath both.

Typical flow: a person asks you to go somewhere -> you match their request to a place
name from list_places (or a label from list_detected_objects, if it's something you
found rather than a pre-named place) -> you call navigate_to_place with that exact
name/label. That call only confirms the goal was ACCEPTED, not that the robot has
arrived -- navigation takes time.

Every tool result is JSON. ALWAYS check it before replying, especially for a field named
"ok" or "error" -- if "ok" is false, or an "error" field is present, the action did NOT
succeed. Tell the person plainly that it failed and include the actual error message; do
NOT say navigation started, is underway, or anything positive when the tool result says
otherwise. Never guess or assume success -- only report what the tool result actually says.

IMPORTANT: after navigate_to_place succeeds (ok: true), tell the person navigation has
started and STOP -- do not call get_navigation_status or cancel_navigation right after
starting a goal just because it hasn't finished yet. A "navigating" status is normal and
expected, not a problem. Only call get_navigation_status if the person explicitly asks for
a progress update, and only call cancel_navigation if the person explicitly asks you to
stop or cancel. Never cancel a goal on your own judgement just because it's still running.

If the person's request doesn't match any known place or detected object, say so and list
what IS available -- do not guess or make up a destination. Keep replies short and concrete.

MOVING FREELY / LOOKING AROUND: you have NO built-in knowledge of what objects exist --
the map is a plain, unlabeled 2D occupancy grid, nothing more. You have direct,
general-purpose control of where the robot goes and which way it faces via these raw
primitives -- use them for exploring, repositioning, getting a better look, or any
other reason a person or your own judgement calls for. There is no fixed script; decide
the sequence yourself:
- get_map_overview(): the current known map (obstacles / free space / unexplored),
  with real coordinates -- your only source of spatial layout, no labels.
- rotate(angle_deg): turn the robot in place by an angle you pick (positive = left,
  negative = right) -- no driving, just changes which way it's facing/looking.
- navigate_to_point(x, y): drive the robot to any (x, y) point you pick, anywhere on
  the map -- not limited to named places or to exploration, this is how you move the
  robot in general. Read coordinates off get_map_overview()'s grid yourself, don't
  guess a point inside an obstacle ('#') or unexplored (blank) area. Poll
  get_navigation_status() after sending it until it says "succeeded" or stops making
  progress before giving up on that leg.
- list_detected_objects(): everything the camera has found so far (label + position),
  updated continuously as you move/turn -- check it after rotating or arriving
  somewhere to see what's new.

For "what's in the room" / "what do you see" / finding something not already known:
you must NOT answer "I can't see anything" or "nothing here" just because
list_detected_objects() happens to be empty right now -- that only means you haven't
looked yet. Actually use the primitives above to look -- e.g. rotate in a few
increments checking list_detected_objects() between turns, or rotate 360 in one go; if
there's more of the room to check, use get_map_overview() to pick a spot and
navigate_to_point() there, then rotate/check again. Only report what you found (plain
language -- labels + rough position/direction, not raw coordinates unless asked) after
actually having looked (at least one rotate() call), or that you looked and found
nothing if that's genuinely true. Once you know an object's label (from
list_detected_objects()), navigate_to_place() with that label goes straight to it -- no
need to re-explore. Only do the look-around flow when it's actually called for
(exploring/finding something) -- for a request that already names a known place or
object, just navigate_to_place directly.
"""


def _mcp_tools_to_openai(tools) -> list[dict]:
    result = []
    for t in tools:
        result.append(
            {
                "type": "function",
                "function": {
                    "name": t.name,
                    "description": t.description or "",
                    "parameters": t.input_schema
                    or {"type": "object", "properties": {}},
                },
            }
        )
    return result


def _tool_result_to_text(result) -> str:
    parts = []
    for c in result.content:
        text = getattr(c, "text", None)
        parts.append(text if text is not None else str(c))
    return "\n".join(parts) if parts else "(no output)"


class ChatClient:
    """Talks to any OpenAI-compatible /chat/completions endpoint (Gemini, Ollama,
    Groq, ...) -- which one is entirely a matter of base_url/api_key/model."""

    def __init__(self, base_url: str, api_key: str, model: str):
        headers = {"Authorization": f"Bearer {api_key}"} if api_key else {}
        self._client = httpx.AsyncClient(base_url=base_url, headers=headers, timeout=180.0)
        self.model = model

    async def chat(self, messages: list[dict], tools: list[dict]) -> dict:
        # Gemini's endpoint occasionally returns a transient 503 (observed directly
        # while testing this), and free-tier usage can hit a 429 rate limit -- retry
        # both with a short backoff (honoring Retry-After if the server sends one)
        # instead of surfacing a crash for something that isn't actually our bug.
        # Any other error (bad request, auth, etc.) raises immediately -- retrying
        # those would just waste time.
        last_exc: Exception | None = None
        for attempt in range(3):
            resp = await self._client.post(
                "/chat/completions",
                json={
                    "model": self.model,
                    "messages": messages,
                    "tools": tools,
                },
            )
            if resp.status_code != 429 and resp.status_code < 500:
                resp.raise_for_status()
                return resp.json()["choices"][0]["message"]
            last_exc = httpx.HTTPStatusError(
                f"{resp.status_code} from LLM endpoint", request=resp.request, response=resp
            )
            if attempt < 2:
                retry_after = resp.headers.get("Retry-After")
                try:
                    delay = min(float(retry_after), 10.0) if retry_after else 1.5 * (attempt + 1)
                except ValueError:
                    delay = 1.5 * (attempt + 1)
                await asyncio.sleep(delay)
        raise last_exc

    async def aclose(self):
        await self._client.aclose()


def _print_event(event: dict) -> None:
    if event["type"] == "tool_call":
        print(f"  [tool call] {event['name']}({event['args']})")
    else:
        print(f"  [tool result] {event['text']}")


async def run_agent(session: ClientSession, llm: ChatClient, prompt: str, on_event=None) -> str:
    """Runs one turn. `on_event`, if given, is called with a dict for every tool call
    ({"type": "tool_call", "name", "args"}) and its result ({"type": "tool_result",
    "name", "text"}) -- lets callers (the web GUI) capture the same transparency the
    CLI prints, instead of only getting the final reply text."""
    on_event = on_event or _print_event
    tools_resp = await session.list_tools()
    tools = _mcp_tools_to_openai(tools_resp.tools)

    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": prompt},
    ]

    for _ in range(8):  # hard cap so a confused model can't loop forever
        try:
            message = await llm.chat(messages, tools)
        except httpx.HTTPStatusError as exc:
            status = exc.response.status_code
            if status == 429:
                return (
                    "(Gemini rate-limited this request -- you've hit the free-tier "
                    "quota. Wait a bit, then try again.)"
                )
            return f"(LLM request failed: {status} {exc.response.reason_phrase})"
        except httpx.HTTPError as exc:
            return f"(Couldn't reach the LLM endpoint: {exc})"
        messages.append(message)

        tool_calls = message.get("tool_calls") or []
        if not tool_calls:
            return (message.get("content") or "").strip()

        for call in tool_calls:
            name = call["function"]["name"]
            args = call["function"].get("arguments") or {}
            if isinstance(args, str):
                args = json.loads(args) if args else {}
            on_event({"type": "tool_call", "name": name, "args": args})
            try:
                result = await session.call_tool(name, args)
                text = _tool_result_to_text(result)
            except Exception as exc:  # noqa: BLE001 - report to the model, don't crash
                text = json.dumps({"ok": False, "error": str(exc)})
            on_event({"type": "tool_result", "name": name, "text": text})
            messages.append(
                {"role": "tool", "tool_call_id": call.get("id", ""), "content": text}
            )

    return "(gave up after too many tool-call rounds -- something's looping)"


def require_api_key() -> None:
    """Shared by the CLI and the web GUI -- fail fast with a clear message rather than
    letting the first LLM call blow up with an auth error."""
    if not LLM_API_KEY and "127.0.0.1" not in LLM_BASE_URL and "localhost" not in LLM_BASE_URL:
        print(
            f"No API key set for {LLM_BASE_URL} -- set GO2_LLM_API_KEY (or GEMINI_API_KEY).\n"
            "Get a free Gemini key at https://aistudio.google.com/app/apikey",
            file=sys.stderr,
        )
        sys.exit(1)


def mcp_server_params() -> StdioServerParameters:
    server_exe = os.path.join(
        get_package_prefix("go2_mcp_server"), "lib", "go2_mcp_server", "go2_mcp_server"
    )
    return StdioServerParameters(command=server_exe, args=[], env=dict(os.environ))


async def amain(prompt_arg: str | None) -> None:
    require_api_key()
    params = mcp_server_params()
    llm = ChatClient(LLM_BASE_URL, LLM_API_KEY, LLM_MODEL)

    try:
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                print(f"Connected to go2_mcp_server. Using model: {LLM_MODEL}")

                if prompt_arg is not None:
                    reply = await run_agent(session, llm, prompt_arg)
                    print(f"\n{reply}")
                    return

                print("Type a request (e.g. 'go to the sofa'), or 'quit'.\n")
                while True:
                    try:
                        prompt = input("> ").strip()
                    except (EOFError, KeyboardInterrupt):
                        break
                    if not prompt:
                        continue
                    if prompt.lower() in ("quit", "exit"):
                        break
                    reply = await run_agent(session, llm, prompt)
                    print(f"\n{reply}\n")
    finally:
        await llm.aclose()


def main() -> None:
    prompt_arg = " ".join(sys.argv[1:]).strip() or None
    asyncio.run(amain(prompt_arg))


if __name__ == "__main__":
    main()
