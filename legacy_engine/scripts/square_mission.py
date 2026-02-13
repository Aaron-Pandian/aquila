import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    drone = System()
    # Connect to the Onboard MAVLink instance we configured
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Square pattern logic (North, East, Down)
    # Note: 'Down' is negative for altitude (e.g., -5m is 5m up)
    square_points = [
        [5, 0, -5],   # 5m North
        [5, 5, -5],   # 5m East
        [0, 5, -5],   # 5m South (back to 0 North)
        [0, 0, -5]    # 5m West (back to 0 East)
    ]

    for point in square_points:
        print(f"-- Moving to: {point}")
        for _ in range(100): 
            await drone.offboard.set_position_ned(PositionNedYaw(point[0], point[1], point[2], 0.0))
            await asyncio.sleep(0.1) # 10Hz frequency

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())