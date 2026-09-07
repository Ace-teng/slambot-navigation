import asyncio
import threading

from .models import GatewayConfig
from .web_server import create_app


def run_server(state, host, port, ready=None, stopped=None):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    runner = None

    async def start():
        nonlocal runner
        from aiohttp import web
        runner = web.AppRunner(create_app(state))
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        if ready:
            ready.set()

    async def stop():
        if runner:
            await runner.cleanup()

    def watch_stop():
        if stopped is not None:
            stopped.wait()
            loop.call_soon_threadsafe(loop.stop)

    try:
        loop.run_until_complete(start())
        threading.Thread(target=watch_stop, daemon=True).start()
        loop.run_forever()
    finally:
        loop.run_until_complete(stop())
        loop.close()


def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from .ros_node import GatewayNode

    rclpy.init(args=args)
    node = GatewayNode()
    host = node.get_parameter("http_host").value
    port = node.get_parameter("http_port").value
    ready = threading.Event()
    stopped = threading.Event()
    thread = threading.Thread(
        target=run_server, args=(node.state, host, port, ready, stopped), daemon=True)
    thread.start()
    try:
        if not ready.wait(5):
            raise RuntimeError("web server did not start")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        thread.join(timeout=5)
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
