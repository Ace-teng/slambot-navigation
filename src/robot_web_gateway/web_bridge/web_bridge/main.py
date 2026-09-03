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
        if stopped:
            stopped.set()

    try:
        loop.run_until_complete(start())
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
    host = node.declare_parameter("http_host", "127.0.0.1").value
    port = node.declare_parameter("http_port", 8080).value
    ready = threading.Event()
    thread = threading.Thread(target=run_server, args=(node.state, host, port, ready), daemon=True)
    thread.start()
    try:
        if not ready.wait(5):
            raise RuntimeError("web server did not start")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
