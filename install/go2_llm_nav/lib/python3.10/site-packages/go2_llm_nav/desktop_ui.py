"""Native desktop window for the Go2 chat GUI -- same page as web_ui.py, but shown in
its own app window (via pywebview, GTK/WebKit2GTK backend) instead of a browser tab. No
address bar, no browser chrome; just the chat.

The HTTP server from web_ui.py still runs underneath (pywebview just points a native
window at http://127.0.0.1:<port>) -- it's on localhost only, not reachable from
anywhere else.

Usage:
    ros2 run go2_llm_nav go2_llm_nav_gui
"""

import asyncio
import os
import threading

# GTK's native Wayland backend can't be verified/inspected the same way X11 windows can
# (confirmed directly: the same window is invisible to `xwininfo` without this, but shows
# up correctly with it) and WebKit2GTK has a rockier history on native Wayland than via
# XWayland -- force X11 (via XWayland) for a rendering path that's actually been
# confirmed to work reliably here. Set before webview's import so GDK picks it up. An
# advanced user can still override by exporting GDK_BACKEND themselves beforehand.
os.environ.setdefault("GDK_BACKEND", "x11")

import webview  # noqa: E402
from aiohttp import web  # noqa: E402
from mcp import ClientSession, stdio_client  # noqa: E402

from go2_llm_nav.agent import (  # noqa: E402
    LLM_API_KEY,
    LLM_BASE_URL,
    LLM_MODEL,
    ChatClient,
    mcp_server_params,
    require_api_key,
)
from go2_llm_nav.web_ui import PORT, make_app


async def _serve(ready: threading.Event, stop_event: asyncio.Event) -> None:
    params = mcp_server_params()
    llm = ChatClient(LLM_BASE_URL, LLM_API_KEY, LLM_MODEL)
    try:
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                app = await make_app(session, llm)
                runner = web.AppRunner(app)
                await runner.setup()
                site = web.TCPSite(runner, "127.0.0.1", PORT)
                await site.start()
                ready.set()
                await stop_event.wait()
                await runner.cleanup()
    finally:
        await llm.aclose()


def main() -> None:
    require_api_key()

    loop = asyncio.new_event_loop()
    ready = threading.Event()

    def run_loop():
        asyncio.set_event_loop(loop)
        stop = asyncio.Event()
        loop.stop_event = stop  # stash it so the closing handler can find it
        loop.run_until_complete(_serve(ready, stop))

    server_thread = threading.Thread(target=run_loop, daemon=True)
    server_thread.start()

    if not ready.wait(timeout=20):
        print("Server didn't come up in time -- check go2_mcp_server / Nav2 are reachable.")
        return

    window = webview.create_window(
        "Go2 Navigator", f"http://127.0.0.1:{PORT}", width=1100, height=750, min_size=(700, 500)
    )

    def on_closing():
        stop = getattr(loop, "stop_event", None)
        if stop is not None:
            loop.call_soon_threadsafe(stop.set)

    window.events.closing += on_closing

    webview.start(gui="gtk")
    server_thread.join(timeout=5)


if __name__ == "__main__":
    main()
