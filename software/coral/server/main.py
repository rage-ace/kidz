import signal
import base64
import asyncio

import cv2
import uvicorn
import numpy as np
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect

from camera import Camera, DebugView
from components import AppState, SingleSlider, XYSlider, HSVSlider


HOSTNAME = "kidz.local"
PORT = 8080


# Set up the camera manager
camera = Camera()
camera.mem.params["render"] = 1  # Enable rendering
# Set up the webserver
app_state = AppState(
    sliders=[
        XYSlider(
            "center-offset",
            (-25, -25),
            (25, 25),
            (1, 1),
            camera.mem.params["frame"]["center_offset"],
        ),
        SingleSlider(
            "crop-radius", 150, 300, 1, camera.mem.params["frame"]["crop_radius"]
        ),
        SingleSlider(
            "robot-radius", 10, 30, 1, camera.mem.params["mask"]["robot_radius"]
        ),
        HSVSlider("orange-lower-bound", camera.mem.params["mask"]["orange"][0]),
        HSVSlider("orange-upper-bound", camera.mem.params["mask"]["orange"][1]),
        HSVSlider("blue-lower-bound", camera.mem.params["mask"]["blue"][0]),
        HSVSlider("blue-upper-bound", camera.mem.params["mask"]["blue"][1]),
        HSVSlider("yellow-lower-bound", camera.mem.params["mask"]["yellow"][0]),
        HSVSlider("yellow-upper-bound", camera.mem.params["mask"]["yellow"][1]),
        HSVSlider("green-lower-bound", camera.mem.params["mask"]["green"][0]),
        HSVSlider("green-upper-bound", camera.mem.params["mask"]["green"][1]),
        SingleSlider(
            "ball-contour-size-max",
            0,
            300,
            1,
            camera.mem.params["contour_size"]["ball"][1],
        ),
        SingleSlider(
            "goal-contour-size-min",
            0,
            150,
            1,
            camera.mem.params["contour_size"]["goal"][0],
        ),
        SingleSlider("test-a", 0, 100, 1, camera.mem.params["test"]["a"]),
        SingleSlider("test-b", 0, 100, 1, camera.mem.params["test"]["b"]),
        SingleSlider("test-c", 0, 100, 1, camera.mem.params["test"]["c"]),
        SingleSlider("test-d", 0, 100, 1, camera.mem.params["test"]["d"]),
        SingleSlider("mask-field", 0, 1, 1, camera.mem.params["mask"]["mask_field"]),
        SingleSlider("render", 0, 1, 1, camera.mem.params["render"]),
    ],
    subfeed_views=[
        DebugView._value2member_map_[view.value]
        for view in camera.mem.params["debug_views"]
    ],
)
app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")


@app.get("/", response_class=HTMLResponse)
async def index(request: Request) -> HTMLResponse:
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "sliders": app_state.get_slider_props(),
            "subfeeds": app_state.get_subfeed_props(),
            "feed_url": f"ws://{HOSTNAME}:{PORT}/feed",
        },
    )


@app.post("/updateslider")
async def update_slider(request: Request) -> None:
    data = await request.json()
    app_state.set_slider_value(html_slider_id=data["id"], value=data["value"])

    # Update camera
    frame_params = camera.mem.params["frame"]
    mask_params = camera.mem.params["mask"]
    contour_params = camera.mem.params["contour_size"]
    frame_params["center_offset"] = app_state.get_slider_value("center-offset")
    frame_params["crop_radius"] = app_state.get_slider_value("crop-radius")
    mask_params["robot_radius"] = app_state.get_slider_value("robot-radius")
    mask_params["mask_field"] = app_state.get_slider_value("mask-field")
    mask_params["orange"][0] = np.array(
        app_state.get_slider_value("orange-lower-bound")
    )
    mask_params["orange"][1] = np.array(
        app_state.get_slider_value("orange-upper-bound")
    )
    mask_params["blue"][0] = np.array(app_state.get_slider_value("blue-lower-bound"))
    mask_params["blue"][1] = np.array(app_state.get_slider_value("blue-upper-bound"))
    mask_params["yellow"][0] = np.array(
        app_state.get_slider_value("yellow-lower-bound")
    )
    mask_params["yellow"][1] = np.array(
        app_state.get_slider_value("yellow-upper-bound")
    )
    mask_params["green"][0] = np.array(app_state.get_slider_value("green-lower-bound"))
    mask_params["green"][1] = np.array(app_state.get_slider_value("green-upper-bound"))
    contour_params["ball"][1] = app_state.get_slider_value("ball-contour-size-max")
    contour_params["goal"][0] = app_state.get_slider_value("goal-contour-size-min")
    camera.mem.params["test"]["a"] = app_state.get_slider_value("test-a")
    camera.mem.params["test"]["b"] = app_state.get_slider_value("test-b")
    camera.mem.params["test"]["c"] = app_state.get_slider_value("test-c")
    camera.mem.params["test"]["d"] = app_state.get_slider_value("test-d")
    camera.mem.params["render"] = app_state.get_slider_value("render")


@app.post("/updatesubfeed")
async def update_subfeed(request: Request) -> None:
    data = await request.json()
    app_state.set_subfeed_value(html_subfeed_id=data["id"], value=data["value"])

    # Update camera
    camera.mem.params["debug_views"] = app_state.get_subfeed_views()


@app.websocket("/feed")
async def video(websocket: WebSocket) -> None:
    # Accept websocket and setup send queue
    await websocket.accept()
    payloads = asyncio.Queue()

    # Start the camera annotation loop
    camera.start_annotating()

    # The camera loop gets frames and sends them out synchronously using the queue
    def camera_loop() -> None:
        def encode_frame(frame: np.ndarray) -> str:
            _, jpeg = cv2.imencode(".jpeg", frame)
            b64_image = base64.b64encode(jpeg).decode("utf-8")
            return b64_image

        while camera.is_annotating():
            # This blocks until a new frame is available
            text, mainfeed, subfeeds = camera.fetch_new_frame()
            payload = {
                "mainfeed": encode_frame(mainfeed),
                "output": text,
                **{
                    f"subfeed{i}": encode_frame(subfeeds[i])
                    for i, view in enumerate(app_state.get_subfeed_views())
                },
            }
            payloads.put_nowait(payload)

    # Run camera loop in separate thread
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, camera_loop)

    # While the camera loop is still running, send out new payloads added to the queue
    while camera.is_annotating():
        while payloads.qsize() > 1:
            await payloads.get()
        payload = await payloads.get()
        try:
            await websocket.send_json(payload)
        except:  # If this errors, the websocket likely closed due to some exception
            break

    # Stop the camera annotation loop
    camera.stop_annotating()


@app.on_event("shutdown")
async def stop_camera_loop() -> None:
    camera.stop()


if __name__ == "__main__":
    # Start the camera manager first to send packets to the Teensy immediately
    camera.start()
    # Then, start the debug webserver
    uvicorn.run("main:app", host="0.0.0.0", port=PORT, log_level="info")
