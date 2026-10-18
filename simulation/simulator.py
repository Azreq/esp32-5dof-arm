from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
import swift
from spatialmath import SE3


URDF_PATH = Path(__file__).resolve().parent / "4dofarm.urdf"

SERVO_OFFSET = 90.0
SERVO_MIN = 0.0
SERVO_MAX = 180.0
JOINT_COUNT = 5

JOINT_NAMES = [
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "end_yaw",
]

INITIAL_ANGLES = np.array([90.0, 90.0, 90.0, 90.0, 90.0])


def load_robot():
    if not URDF_PATH.exists():
        raise FileNotFoundError(f"File URDF tidak ditemukan: {URDF_PATH}")

    robot = rtb.ERobot.URDF(str(URDF_PATH))
    arm = robot.ets(end="tool_tip")

    if arm.n != JOINT_COUNT:
        raise RuntimeError(
            f"Model seharusnya memiliki {JOINT_COUNT} joint aktif, "
            f"tetapi ditemukan {arm.n}. Periksa 4dofarm.urdf."
        )

    return robot, arm


def servo_to_radians(angles_deg):
    angles_deg = np.asarray(angles_deg, dtype=float)

    if angles_deg.shape != (JOINT_COUNT,):
        raise ValueError("Masukkan tepat 5 sudut servo: base, shoulder, elbow, wrist, end joint.")

    if not np.all(np.isfinite(angles_deg)):
        raise ValueError("Sudut servo harus berupa angka yang valid.")

    if np.any(angles_deg < SERVO_MIN) or np.any(angles_deg > SERVO_MAX):
        raise ValueError("Sudut servo harus berada pada rentang 0–180 derajat.")

    return np.radians(angles_deg - SERVO_OFFSET)


def radians_to_servo(angles_rad):
    return np.degrees(np.asarray(angles_rad, dtype=float)) + SERVO_OFFSET


def validate_joint_limits(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (JOINT_COUNT,) or not np.all(np.isfinite(q)):
        return False

    limits = np.asarray(arm.qlim, dtype=float)
    if limits.shape[1] < JOINT_COUNT:
        raise RuntimeError("Batas joint URDF tidak terbaca dengan benar.")

    lower = limits[0, :JOINT_COUNT]
    upper = limits[1, :JOINT_COUNT]

    if np.any(q < lower - 1e-6) or np.any(q > upper + 1e-6):
        return False

    servo_angles = radians_to_servo(q)
    return bool(
        np.all(servo_angles >= SERVO_MIN - 1e-6)
        and np.all(servo_angles <= SERVO_MAX + 1e-6)
    )


def update_visualization(q):
    full_q = np.array(robot.q, dtype=float, copy=True)
    full_q[:JOINT_COUNT] = np.asarray(q, dtype=float)
    robot.q = full_q
    env.step()


def print_joint_angles(angles):
    for name, angle in zip(JOINT_NAMES, angles):
        print(f"{name}: {angle:.2f}°")


def run_fk(base_deg, shoulder_deg, elbow_deg, wrist_deg, end_deg):
    try:
        servo_angles = [base_deg, shoulder_deg, elbow_deg, wrist_deg, end_deg]
        q = servo_to_radians(servo_angles)

        if not validate_joint_limits(q):
            print("[ERROR] Salah satu sudut berada di luar batas joint URDF.")
            return

        transform = arm.fkine(q)
        position = np.asarray(transform.t).reshape(3) * 1000.0
        update_visualization(q)

        print("\n--- FORWARD KINEMATICS (5-DOF) ---")
        print_joint_angles(servo_angles)
        print(f"X = {position[0]:.2f} mm\nY = {position[1]:.2f} mm\nZ = {position[2]:.2f} mm")

    except (ValueError, TypeError, RuntimeError) as error:
        print(f"[ERROR] {error}")


def run_ik(target_x_mm, target_y_mm, target_z_mm):
    target = np.asarray([target_x_mm, target_y_mm, target_z_mm], dtype=float) / 1000.0

    if not np.all(np.isfinite(target)):
        print("[ERROR] Koordinat target tidak valid.")
        return

    current_q = np.asarray(robot.q, dtype=float).reshape(-1)[:JOINT_COUNT]
    target_transform = SE3(float(target[0]), float(target[1]), float(target[2]))

    try:
        solution = arm.ikine_LM(
            target_transform,
            q0=current_q,
            mask=[1, 1, 1, 0, 0, 0],
            ilimit=500,
            slimit=200,
            tol=1e-7,
        )

        if not solution.success:
            print("[ERROR] IK gagal. Target mungkin di luar jangkauan atau solver tidak menemukan solusi.")
            return

        q_solution = np.asarray(solution.q, dtype=float).reshape(-1)
        if not validate_joint_limits(q_solution):
            print("[ERROR] Solusi IK melampaui batas joint atau rentang servo 0–180°.")
            return

        actual_position = np.asarray(arm.fkine(q_solution).t).reshape(3)
        error = float(np.linalg.norm(actual_position - target))
        if error > 5e-4:
            print(f"[ERROR] Target belum tercapai dengan akurasi cukup. Error: {error * 1000:.4f} mm")
            return

        servo_angles = radians_to_servo(q_solution)
        update_visualization(q_solution)

        print("\n--- INVERSE KINEMATICS (5-DOF) ---")
        print(f"Target: X={target_x_mm:.2f} mm, Y={target_y_mm:.2f} mm, Z={target_z_mm:.2f} mm")
        print_joint_angles(servo_angles)
        print(f"Error posisi: {error * 1000:.6f} mm")

    except (ValueError, TypeError, RuntimeError) as error:
        print(f"[ERROR] IK gagal: {error}")


try:
    robot, arm = load_robot()
except Exception as error:
    raise SystemExit(f"[ERROR] Gagal memuat model robot: {error}")

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

try:
    env.browser.background = [0.9, 0.9, 0.9]
except Exception:
    pass

initial_q = servo_to_radians(INITIAL_ANGLES)
if not validate_joint_limits(initial_q):
    env.close()
    raise SystemExit("[ERROR] Posisi awal melampaui batas joint pada URDF.")

update_visualization(initial_q)


if __name__ == "__main__":
    print("SIMULATOR ROBOT ARM 5-DOF")
    print("1. Forward Kinematics")
    print("2. Inverse Kinematics")
    print("3. Keluar")

    while True:
        print("\nPilih mode:")
        print("1. Forward Kinematics")
        print("2. Inverse Kinematics")
        print("3. Keluar")
        choice = input("Pilihan (1/2/3): ").strip()

        if choice == "1":
            print("\nMasukkan sudut servo 0–180 derajat.")
            try:
                angles = [
                    float(input("Sudut Base (0–180): ")),
                    float(input("Sudut Shoulder (0–180): ")),
                    float(input("Sudut Elbow (0–180): ")),
                    float(input("Sudut Wrist (0–180): ")),
                    float(input("Sudut End Joint (0–180): ")),
                ]
                run_fk(*angles)
            except ValueError:
                print("[ERROR] Masukkan angka yang valid.")

        elif choice == "2":
            print("\nIK menargetkan posisi titik tool_tip; orientasi ujung tidak dikunci.")
            try:
                x = float(input("Target X (mm): "))
                y = float(input("Target Y (mm): "))
                z = float(input("Target Z (mm): "))
                run_ik(x, y, z)
            except ValueError:
                print("[ERROR] Masukkan angka yang valid.")

        elif choice == "3":
            print("Simulator selesai.")
            env.close()
            break

        else:
            print("[ERROR] Pilihan tidak tersedia.")
