import time
from camera import Camera


DEBUG = False


def main():
    camera = Camera()
    camera.start()

    try:
        while True:
            if DEBUG:
                t = camera.mem.loop_trackers
                text = "             Read  Mask  Ball  Goal  Send Render\n"
                text += (
                    f"FPS       : {t['fetch_frame'].mean_fps():5.1f} "
                    f"{t['preprocess_frame'].mean_fps():5.1f} "
                    f"{t['detect_ball'].mean_fps():5.1f} "
                    f"{t['detect_goals'].mean_fps():5.1f} "
                    f"{t['send_payload'].mean_fps():5.1f} "
                    f"{t['annotate_frame'].mean_fps():5.1f} (FPS)\n"
                )
                text += (
                    f"Loop Time : {t['fetch_frame'].mean_loop_time():5.1f} "
                    f"{t['preprocess_frame'].mean_loop_time():5.1f} "
                    f"{t['detect_ball'].mean_loop_time():5.1f} "
                    f"{t['detect_goals'].mean_loop_time():5.1f} "
                    f"{t['send_payload'].mean_loop_time():5.1f} "
                    f"{t['annotate_frame'].mean_loop_time():5.1f} (ms)\n\n"
                )
                print(text)
            time.sleep(0.2)
    except KeyboardInterrupt:
        camera.stop()


if __name__ == "__main__":
    main()
