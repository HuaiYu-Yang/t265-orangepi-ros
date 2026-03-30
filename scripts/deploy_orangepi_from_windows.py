#!/usr/bin/env python3
import argparse
import os
import tempfile
from pathlib import Path

import paramiko


def run(ssh, command, timeout=600):
    stdin, stdout, stderr = ssh.exec_command(command, timeout=timeout)
    out = stdout.read().decode(errors="ignore")
    err = stderr.read().decode(errors="ignore")
    if out:
        print(out, end="")
    if err:
        print(err, end="")
    exit_code = stdout.channel.recv_exit_status()
    if exit_code != 0:
        raise RuntimeError(f"remote command failed: {command}")


def rebuild_bundle(parts_dir):
    parts_dir = Path(parts_dir)
    parts = sorted(parts_dir.glob("librealsense-2.50.0-offline-arm64.tar.gz.part*"))
    if not parts:
        raise FileNotFoundError(f"No bundle parts found in {parts_dir}")

    tmp_dir = Path(tempfile.mkdtemp(prefix="t265_bundle_"))
    bundle_path = tmp_dir / "librealsense-2.50.0-offline-arm64.tar.gz"
    with bundle_path.open("wb") as dst:
        for part in parts:
            dst.write(part.read_bytes())
    return bundle_path


def main():
    repo_root = Path(__file__).resolve().parent.parent

    parser = argparse.ArgumentParser(description="Deploy split offline T265 bundle and ROS node to Orange Pi")
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument(
        "--bundle-parts-dir",
        default=str(repo_root / "bundle"),
        help="Directory containing split bundle parts",
    )
    parser.add_argument(
        "--node",
        default=str(repo_root / "ros" / "t265_odom_node.py"),
        help="Path to ROS node file",
    )
    args = parser.parse_args()

    bundle_path = rebuild_bundle(args.bundle_parts_dir)
    bundle_name = bundle_path.name
    remote_home = f"/home/{args.user}"
    remote_bundle = f"{remote_home}/{bundle_name}"
    remote_src_dir = f"{remote_home}/librealsense-2.50.0"
    remote_node = f"{remote_home}/t265_odom_node.py"

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(args.host, username=args.user, password=args.password, timeout=20)

    sftp = ssh.open_sftp()
    try:
        print(f"[deploy] upload bundle -> {remote_bundle}")
        sftp.put(str(bundle_path), remote_bundle)
        print(f"[deploy] upload node -> {remote_node}")
        sftp.put(args.node, remote_node)
    finally:
        sftp.close()

    run(ssh, f"bash -lc 'cd {remote_home} && rm -rf librealsense-2.50.0 && tar -xzf {bundle_name}'", timeout=300)
    run(ssh, f"bash -lc 'cd {remote_src_dir} && bash build_orangepi_offline.sh'", timeout=3600)
    print("[deploy] done")
    print(f"[deploy] run on Orange Pi: source /opt/ros/noetic/setup.bash && python3 {remote_node}")
    ssh.close()


if __name__ == "__main__":
    main()
