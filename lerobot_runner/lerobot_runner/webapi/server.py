"""WebAPI server for policy runner control."""
import asyncio
import json
import threading
from typing import TYPE_CHECKING, Callable, Dict, Any, Optional

try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    web = None

if TYPE_CHECKING:
    from ..policy_runner_node import PolicyRunnerNode


class WebAPIServer:
    """
    Async WebAPI server for controlling the policy runner.

    Provides REST API endpoints for:
    - Status monitoring
    - Model loading/unloading
    - Inference start/stop
    - Task changes
    - Configuration
    """

    def __init__(self, node: 'PolicyRunnerNode', port: int = 8083):
        """
        Initialize WebAPI server.

        Args:
            node: PolicyRunnerNode instance
            port: Port to listen on
        """
        self._node = node
        self._port = port
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._runner: Optional[web.AppRunner] = None

    def start(self):
        """Start the WebAPI server in a background thread."""
        if not HAS_AIOHTTP:
            self._node.get_logger().error("aiohttp not installed. WebAPI disabled.")
            return

        def run_server():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            app = web.Application()
            self._setup_routes(app)

            self._runner = web.AppRunner(app)
            self._loop.run_until_complete(self._runner.setup())
            site = web.TCPSite(self._runner, '0.0.0.0', self._port)
            self._loop.run_until_complete(site.start())
            self._loop.run_forever()

        self._thread = threading.Thread(target=run_server, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the WebAPI server."""
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)

    def _setup_routes(self, app: web.Application):
        """Setup API routes."""
        # Status
        app.router.add_get('/api/status', self._handle_status)
        app.router.add_get('/api/config', self._handle_get_config)

        # Models
        app.router.add_get('/api/models', self._handle_list_models)
        app.router.add_get('/api/policies', self._handle_list_policies)

        # Control
        app.router.add_post('/api/load', self._handle_load)
        app.router.add_post('/api/unload', self._handle_unload)
        app.router.add_post('/api/start', self._handle_start)
        app.router.add_post('/api/stop', self._handle_stop)
        app.router.add_post('/api/task', self._handle_set_task)

    # === Handlers ===

    async def _handle_status(self, request: web.Request) -> web.Response:
        """GET /api/status - Get current status."""
        status = self._node.get_status()
        return web.json_response(status)

    async def _handle_get_config(self, request: web.Request) -> web.Response:
        """GET /api/config - Get current configuration."""
        config = self._node.get_config()
        return web.json_response(config)

    async def _handle_list_models(self, request: web.Request) -> web.Response:
        """GET /api/models - List available models."""
        models = self._node.list_models()
        return web.json_response(models)

    async def _handle_list_policies(self, request: web.Request) -> web.Response:
        """GET /api/policies - List available policy types."""
        from ..policy import POLICY_REGISTRY
        policies = list(POLICY_REGISTRY.keys())
        return web.json_response({"policies": policies})

    async def _handle_load(self, request: web.Request) -> web.Response:
        """POST /api/load - Load a model."""
        try:
            data = await request.json()
        except:
            return web.json_response(
                {"success": False, "error": "Invalid JSON"},
                status=400
            )

        checkpoint_path = data.get('checkpoint_path')
        if not checkpoint_path:
            return web.json_response(
                {"success": False, "error": "checkpoint_path is required"},
                status=400
            )

        policy_type = data.get('policy_type')
        device = data.get('device')

        result = self._node.load_model(checkpoint_path, policy_type, device)
        status_code = 200 if result.get('success') else 500
        return web.json_response(result, status=status_code)

    async def _handle_unload(self, request: web.Request) -> web.Response:
        """POST /api/unload - Unload current model."""
        result = self._node.unload_model()
        return web.json_response(result)

    async def _handle_start(self, request: web.Request) -> web.Response:
        """POST /api/start - Start inference."""
        try:
            data = await request.json()
        except:
            data = {}

        task = data.get('task')
        inference_rate = data.get('inference_rate')
        publish_rate = data.get('publish_rate')

        result = self._node.start_inference(task, inference_rate, publish_rate)
        status_code = 200 if result.get('success') else 400
        return web.json_response(result, status=status_code)

    async def _handle_stop(self, request: web.Request) -> web.Response:
        """POST /api/stop - Stop inference."""
        result = self._node.stop_inference()
        return web.json_response(result)

    async def _handle_set_task(self, request: web.Request) -> web.Response:
        """POST /api/task - Update task description."""
        try:
            data = await request.json()
        except:
            return web.json_response(
                {"success": False, "error": "Invalid JSON"},
                status=400
            )

        task = data.get('task')
        if not task:
            return web.json_response(
                {"success": False, "error": "task is required"},
                status=400
            )

        result = self._node.set_task(task)
        return web.json_response(result)
