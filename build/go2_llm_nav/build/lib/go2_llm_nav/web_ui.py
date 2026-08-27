"""Local web GUI for the Go2 natural-language navigation agent -- a chat page you open
in a browser instead of typing in a terminal. Same model, same tools, same system
prompt as the CLI (go2_llm_nav.agent); this just wraps it in aiohttp and serves a
single self-contained HTML page.

Usage:
    ros2 run go2_llm_nav go2_llm_nav_web              # serves http://127.0.0.1:8765
    GO2_WEB_PORT=9000 ros2 run go2_llm_nav go2_llm_nav_web
"""

import asyncio
import json
import os
import webbrowser

from aiohttp import web
from mcp import ClientSession, stdio_client

from go2_llm_nav.agent import (
    LLM_API_KEY,
    LLM_BASE_URL,
    LLM_MODEL,
    ChatClient,
    _tool_result_to_text,
    mcp_server_params,
    require_api_key,
    run_agent,
)

PORT = int(os.environ.get("GO2_WEB_PORT", "8765"))

INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 Navigator</title>
<style>
  :root {
    --bg: #0f1115; --panel: #171a21; --border: #262b36; --text: #e6e8ec;
    --muted: #8b93a3; --accent: #4f8cff; --user-bubble: #2a3550; --bot-bubble: #1c202a;
    --ok: #3ddc84; --err: #ff5d5d;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg); color: var(--text); height: 100vh; display: flex;
  }
  #sidebar {
    width: 240px; flex-shrink: 0; background: var(--panel); border-right: 1px solid var(--border);
    padding: 16px; display: flex; flex-direction: column; gap: 16px;
  }
  #sidebar h1 { font-size: 16px; margin: 0 0 4px; }
  #sidebar .sub { font-size: 12px; color: var(--muted); margin-bottom: 8px; }
  #status { font-size: 13px; padding: 10px; border-radius: 8px; background: var(--bot-bubble); border: 1px solid var(--border); }
  #status .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: var(--muted); margin-right: 6px; }
  #status.navigating .dot { background: var(--accent); }
  #status.idle .dot { background: var(--ok); }
  #places { display: flex; flex-direction: column; gap: 6px; overflow-y: auto; }
  .place-btn {
    text-align: left; background: var(--bot-bubble); color: var(--text); border: 1px solid var(--border);
    border-radius: 8px; padding: 8px 10px; cursor: pointer; font-size: 13px;
  }
  .place-btn:hover { border-color: var(--accent); }
  .place-btn .name { font-weight: 600; display: block; }
  .place-btn .desc { color: var(--muted); font-size: 11px; }
  #main { flex: 1; display: flex; flex-direction: column; min-width: 0; }
  #chat { flex: 1; overflow-y: auto; padding: 20px; display: flex; flex-direction: column; gap: 14px; }
  .msg { max-width: 70%; padding: 10px 14px; border-radius: 12px; line-height: 1.4; font-size: 14px; white-space: pre-wrap; }
  .msg.user { align-self: flex-end; background: var(--user-bubble); }
  .msg.bot { align-self: flex-start; background: var(--bot-bubble); border: 1px solid var(--border); }
  .msg.system { align-self: center; color: var(--muted); font-size: 12px; }
  .tool-log { align-self: flex-start; max-width: 80%; font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 12px; color: var(--muted); background: #11141a; border: 1px solid var(--border); border-radius: 8px; padding: 8px 10px; white-space: pre-wrap; }
  .tool-log .ok { color: var(--ok); }
  .tool-log .err { color: var(--err); }
  #inputRow { display: flex; gap: 8px; padding: 16px; border-top: 1px solid var(--border); }
  #inputBox { flex: 1; background: var(--bot-bubble); color: var(--text); border: 1px solid var(--border); border-radius: 10px; padding: 12px 14px; font-size: 14px; resize: none; }
  #inputBox:focus { outline: none; border-color: var(--accent); }
  #sendBtn { background: var(--accent); color: white; border: none; border-radius: 10px; padding: 0 20px; font-size: 14px; cursor: pointer; }
  #sendBtn:disabled { opacity: 0.5; cursor: default; }
  .spinner { display: inline-block; width: 10px; height: 10px; border: 2px solid var(--muted); border-top-color: var(--accent); border-radius: 50%; animation: spin 0.7s linear infinite; }
  @keyframes spin { to { transform: rotate(360deg); } }
</style>
</head>
<body>
  <div id="sidebar">
    <div>
      <h1>Go2 Navigator</h1>
      <div class="sub" id="modelLabel">connecting...</div>
    </div>
    <div id="status" class="idle"><span class="dot"></span><span id="statusText">idle</span></div>
    <div class="sub">Known places (click to send)</div>
    <div id="places"></div>
  </div>
  <div id="main">
    <div id="chat"></div>
    <div id="inputRow">
      <textarea id="inputBox" rows="1" placeholder="Ask something, or tell it where to go..."></textarea>
      <button id="sendBtn">Send</button>
    </div>
  </div>

<script>
const chatEl = document.getElementById('chat');
const inputEl = document.getElementById('inputBox');
const sendBtn = document.getElementById('sendBtn');
const placesEl = document.getElementById('places');
const statusEl = document.getElementById('status');
const statusTextEl = document.getElementById('statusText');
const modelLabelEl = document.getElementById('modelLabel');

function addMsg(text, cls) {
  const d = document.createElement('div');
  d.className = 'msg ' + cls;
  d.textContent = text;
  chatEl.appendChild(d);
  chatEl.scrollTop = chatEl.scrollHeight;
  return d;
}

function addToolLog(events) {
  if (!events || events.length === 0) return;
  const d = document.createElement('div');
  d.className = 'tool-log';
  let lines = [];
  for (const e of events) {
    if (e.type === 'tool_call') {
      lines.push(`[tool call] ${e.name}(${JSON.stringify(e.args)})`);
    } else {
      let cls = 'ok';
      try { if (JSON.parse(e.text).ok === false) cls = 'err'; } catch (_) {}
      lines.push(`[tool result] ${e.text}`);
    }
  }
  d.textContent = lines.join('\\n');
  chatEl.appendChild(d);
  chatEl.scrollTop = chatEl.scrollHeight;
}

async function sendMessage(text) {
  if (!text.trim()) return;
  addMsg(text, 'user');
  inputEl.value = '';
  sendBtn.disabled = true;
  const thinking = addMsg('...', 'bot');
  try {
    const resp = await fetch('/api/chat', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({message: text}),
    });
    const data = await resp.json();
    thinking.remove();
    addToolLog(data.events);
    addMsg(data.reply || '(no reply)', 'bot');
  } catch (err) {
    thinking.remove();
    addMsg('(request failed: ' + err + ')', 'bot');
  } finally {
    sendBtn.disabled = false;
    inputEl.focus();
  }
}

sendBtn.addEventListener('click', () => sendMessage(inputEl.value));
inputEl.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    sendMessage(inputEl.value);
  }
});

async function loadPlaces() {
  try {
    const resp = await fetch('/api/places');
    const data = await resp.json();
    placesEl.innerHTML = '';
    for (const p of data.places || []) {
      const b = document.createElement('button');
      b.className = 'place-btn';
      b.innerHTML = `<span class="name">${p.name}</span><span class="desc">${p.description}</span>`;
      b.addEventListener('click', () => sendMessage('go to the ' + p.name));
      placesEl.appendChild(b);
    }
  } catch (err) {
    placesEl.textContent = '(could not load places)';
  }
}

async function pollStatus() {
  try {
    const resp = await fetch('/api/status');
    const data = await resp.json();
    const state = data.state || 'unknown';
    statusEl.className = state;
    let text = state;
    if (data.target) text += ` -> ${data.target}`;
    if (typeof data.distance_remaining_m === 'number') text += ` (${data.distance_remaining_m.toFixed(1)}m left)`;
    statusTextEl.textContent = text;
  } catch (err) {
    statusTextEl.textContent = 'unavailable';
  }
}

async function loadInfo() {
  try {
    const resp = await fetch('/api/info');
    const data = await resp.json();
    modelLabelEl.textContent = 'model: ' + data.model;
  } catch (err) {
    modelLabelEl.textContent = 'model: unknown';
  }
}

addMsg('Connected. Ask something, or tell the robot where to go.', 'system');
loadInfo();
loadPlaces();
pollStatus();
setInterval(pollStatus, 3000);
inputEl.focus();
</script>
</body>
</html>
"""


async def make_app(session: ClientSession, llm: ChatClient) -> web.Application:
    async def handle_index(request):
        return web.Response(text=INDEX_HTML, content_type="text/html")

    async def handle_chat(request):
        data = await request.json()
        prompt = (data.get("message") or "").strip()
        if not prompt:
            return web.json_response({"error": "empty message"}, status=400)
        events = []
        reply = await run_agent(session, llm, prompt, on_event=events.append)
        return web.json_response({"reply": reply, "events": events})

    async def handle_places(request):
        result = await session.call_tool("list_places", {})
        text = _tool_result_to_text(result)
        places = []
        for chunk in _split_json_objects(text):
            try:
                places.append(json.loads(chunk))
            except json.JSONDecodeError:
                continue
        return web.json_response({"places": places})

    async def handle_status(request):
        result = await session.call_tool("get_navigation_status", {})
        text = _tool_result_to_text(result)
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            data = {"state": "unknown", "raw": text}
        return web.json_response(data)

    async def handle_info(request):
        return web.json_response({"model": LLM_MODEL, "base_url": LLM_BASE_URL})

    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_post("/api/chat", handle_chat)
    app.router.add_get("/api/places", handle_places)
    app.router.add_get("/api/status", handle_status)
    app.router.add_get("/api/info", handle_info)
    return app


def _split_json_objects(text: str) -> list[str]:
    """list_places's tool result is one or more pretty-printed JSON objects
    concatenated with newlines (not a JSON array) -- split them back out by brace
    depth so each can be parsed individually."""
    chunks = []
    depth = 0
    start = 0
    for i, ch in enumerate(text):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                chunks.append(text[start : i + 1])
    return chunks


async def amain() -> None:
    require_api_key()
    params = mcp_server_params()
    llm = ChatClient(LLM_BASE_URL, LLM_API_KEY, LLM_MODEL)

    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            app = await make_app(session, llm)
            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, "127.0.0.1", PORT)
            await site.start()
            url = f"http://127.0.0.1:{PORT}"
            print(f"Go2 chat GUI running at {url} -- open it in a browser. Ctrl-C to stop.")
            try:
                webbrowser.open(url)
            except Exception:
                pass
            try:
                await asyncio.Event().wait()
            finally:
                await runner.cleanup()
                await llm.aclose()


def main() -> None:
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
