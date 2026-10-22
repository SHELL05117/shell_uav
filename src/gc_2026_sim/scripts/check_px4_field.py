#!/usr/bin/env python3
"""Check a Classic world and Iris spawn without starting PX4 or a GUI."""

import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def run(command, env, timeout):
    try:
        result = subprocess.run(
            command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        output = error.output or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        raise RuntimeError("Timed out: {}\n{}".format(
            " ".join(command), output))
    if result.returncode:
        raise RuntimeError("{}\n{}".format(" ".join(command), result.stdout))
    return result.stdout


def check(world, px4_root, timeout, wait_world):
    world_name = ET.parse(str(world)).getroot().find("world").get("name")
    models = px4_root / "Tools/sitl_gazebo/models"
    plugins = px4_root / "build/px4_sitl_default/build_gazebo"
    iris = models / "iris/iris.sdf"
    if not iris.is_file():
        raise RuntimeError("Iris model not found: {}".format(iris))

    env = os.environ.copy()
    for key, directory in (
            ("GAZEBO_MODEL_PATH", models),
            ("GAZEBO_PLUGIN_PATH", plugins),
            ("LD_LIBRARY_PATH", plugins)):
        env[key] = str(directory) + os.pathsep + env.get(key, "")
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:{}".format(port)

    print("WORLD={} NAME={} MASTER={}".format(
        world, world_name, env["GAZEBO_MASTER_URI"]), flush=True)
    print(run(["gz", "sdf", "-k", str(world)], env, timeout), flush=True)
    with tempfile.TemporaryFile(mode="w+") as log:
        server = subprocess.Popen(
            ["gzserver", "--verbose", "-u", str(world)], env=env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if wait_world:
                deadline = time.monotonic() + timeout
                factory = "/gazebo/{}/factory".format(world_name)
                while time.monotonic() < deadline:
                    if server.poll() is not None:
                        raise RuntimeError("gzserver exited before world was ready")
                    try:
                        if factory in run(["gz", "topic", "-l"], env, 2):
                            break
                    except RuntimeError:
                        pass
                    time.sleep(0.2)
                else:
                    raise RuntimeError("World factory did not become ready")
                print("WORLD_READY", flush=True)
            # Match Tools/sitl_run.sh: no explicit --world-name argument.
            started = time.monotonic()
            output = run([
                "gz", "model", "--verbose", "--spawn-file=" + str(iris),
                "--model-name=iris", "-x", "0", "-y", "0", "-z", "0.4"],
                env, timeout)
            print(output, flush=True)
            print("SPAWN_RETURNED seconds={:.2f}".format(
                time.monotonic() - started), flush=True)
            info = run([
                "gz", "model", "--world-name=" + world_name,
                "--model-name=iris", "--info"], env, timeout)
            if 'name: "iris"' not in info:
                raise RuntimeError("Iris spawn returned but model is absent")
            print("IRIS_PRESENT", flush=True)
        finally:
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
            log.seek(0)
            print("GZSERVER_LOG:\n" + log.read(), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-root", type=Path,
                        default=Path.home() / "PX4-Autopilot")
    parser.add_argument("--world", type=Path, default=(
        Path(__file__).resolve().parent.parent
        / "gazebo/worlds/gc_2026_field.world"))
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--wait-world", action="store_true",
                        help="Wait for the world before spawning (diagnostic control)")
    args = parser.parse_args()
    try:
        check(args.world.resolve(), args.px4_root.resolve(), args.timeout,
              args.wait_world)
    except (RuntimeError, subprocess.TimeoutExpired) as error:
        parser.exit(1, "SMOKE_TEST_FAILED: {}\n".format(error))
    print("SMOKE_TEST_PASSED")


if __name__ == "__main__":
    main()
