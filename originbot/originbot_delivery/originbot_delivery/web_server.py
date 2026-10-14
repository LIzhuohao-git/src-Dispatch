#!/usr/bin/env python3

import functools
import http.server
import os
import threading

import rclpy
from rclpy.node import Node


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        return


class WebServer(Node):
    def __init__(self):
        super().__init__("delivery_web_server")
        self.declare_parameter("port", 8080)
        self.declare_parameter("www_root", "")

        port = int(self.get_parameter("port").value)
        www_root = self.get_parameter("www_root").value
        if not www_root:
            from ament_index_python.packages import get_package_share_directory

            www_root = os.path.join(
                get_package_share_directory("originbot_delivery"),
                "static",
            )

        handler = functools.partial(QuietHandler, directory=www_root)
        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, name="delivery-web", daemon=True
        )
        self.thread.start()
        self.get_logger().info(f"Web UI listening on http://0.0.0.0:{port}")

    def shutdown(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
