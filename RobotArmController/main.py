"""
This program is the main controller running on a PC.
Controls the robot arm and communicates with Lego Technic Hubs via Bluetooth.

This file mostly handles Bluetooth and Arduino connections. The higher level logic is handled in 
the RobotArmController/RobotArmController.py file.
"""

from time import sleep
import asyncio
from contextlib import suppress
from bleak import BleakScanner, BleakClient
from RobotArmController import control_robot_arm

SW_VERSION = "0.2.0"  # Software version of this program
PYBRICKS_COMMAND_EVENT_CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
HUB_NAME_SHOULDER_CONTROLLER = "Pybricks Hub"
HUB_NAME_LOWER_ARM_CONTROLLER = "Pybricks hub 2"


async def main():
    print(f"Starting robot arm main controller, version {SW_VERSION}.")

    main_task = asyncio.current_task()

    def handle_bluetooth_disconnect(_):
        print("A Lego hub was disconnected.")

        # If the hub disconnects before this program is done,
        # cancel this program so it doesn't get stuck waiting
        # forever.
        if not main_task.done():
            main_task.cancel()


    lego_hub_ready_event_1 = asyncio.Event() # To tell when the hub is ready to receive data.
    lego_hub_ready_event_2 = asyncio.Event() # To tell when the hub is ready to receive data.


    def handle_rx_1(_, data: bytearray):
        if data[0] == 0x01:  # "write stdout" event (0x01)
            payload = data[1:]

            if payload == b"rdy":
                print("Received 'rdy' from the first hub.")
                lego_hub_ready_event_1.set()
            else:
                print("Received:", payload)
                # TODO: Handle the received data here.


    def handle_rx_2(_, data: bytearray):
        if data[0] == 0x01:  # "write stdout" event (0x01)
            payload = data[1:]

            if payload == b"rdy":
                print("Received 'rdy' from the second hub.")
                lego_hub_ready_event_2.set()
            else:
                print("Received:", payload)
                # TODO: Handle the received data here.


    print("Searching for the first hub...")
    bluetooth_device_1 = await BleakScanner.find_device_by_name(HUB_NAME_SHOULDER_CONTROLLER) # Do a Bluetooth scan to find the first hub.

    if bluetooth_device_1 is None:
        print(f"Could not find hub with name: {HUB_NAME_SHOULDER_CONTROLLER}")
        return

    async with BleakClient(bluetooth_device_1, handle_bluetooth_disconnect) as bluetooth_client_1: # Connect to the first hub.
        print(f"Connected to {bluetooth_device_1.name}")
        await bluetooth_client_1.start_notify(PYBRICKS_COMMAND_EVENT_CHAR_UUID, handle_rx_1) # Subscribe to notifications from the first hub.

        print("Searching for the second hub...")
        bluetooth_device_2 = await BleakScanner.find_device_by_name(HUB_NAME_LOWER_ARM_CONTROLLER) # Do a Bluetooth scan to find the second hub.

        if bluetooth_device_2 is None:
            print(f"Could not find hub with name: {HUB_NAME_LOWER_ARM_CONTROLLER}")
            return

        async with BleakClient(bluetooth_device_2, handle_bluetooth_disconnect) as bluetooth_client_2: # Connect to the second hub.
            print(f"Connected to {bluetooth_device_2.name}")
            await bluetooth_client_2.start_notify(PYBRICKS_COMMAND_EVENT_CHAR_UUID, handle_rx_2) # Subscribe to notifications from the second hub.

            # Tell user to start program on the hub.
            print("Start the programs on the two Lego hubs now with the button.")

            await lego_hub_ready_event_1.wait()  # Wait for hub to say that it is ready to receive data.
            lego_hub_ready_event_1.clear()       # Prepare for the next ready event.
            await lego_hub_ready_event_2.wait()  # Wait for hub to say that it is ready to receive data.
            lego_hub_ready_event_2.clear()       # Prepare for the next ready event.
            print("Both hubs are ready to receive data.")

            async def send(data):
                """Send data to the two hubs."""
                # Send the same data to both hubs.
                await bluetooth_client_1.write_gatt_char(
                    PYBRICKS_COMMAND_EVENT_CHAR_UUID,
                    b"\x06" + data,  # prepend "write stdin" command (0x06)
                    response=True
                ) # This code is based on the example at https://pybricks.com/projects/tutorials/wireless/hub-to-device/pc-communication/
                await bluetooth_client_2.write_gatt_char(
                    PYBRICKS_COMMAND_EVENT_CHAR_UUID,
                    b"\x06" + data,  # prepend "write stdin" command (0x06)
                    response=True
                )


            # Defer to RobotArmController.py for the actual robot arm control logic, and provide it the send() function.
            await control_robot_arm(send)

            print("Done.")

    # Hub disconnects here when async with block exits.


# Run the main async program.
if __name__ == "__main__":
    with suppress(asyncio.CancelledError):
        while True:
            asyncio.run(main())
            print("Main loop exited, probably because the hub was disconnected. Restarting main loop in 5 seconds...")
            sleep(5)
