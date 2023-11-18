import requests
from requests.exceptions import RequestException

# Replace with your ESP32's IP address
esp32_ip = "192.168.50.149"  # Change this to your ESP32's IP address

# Define the URLs to control the motor
pull_url = f"http://{esp32_ip}/PULL"
push_url = f"http://{esp32_ip}/PUSH"
stop_url = f"http://{esp32_ip}/STOP"

def control_motor(action_url):
    try:
        response = requests.get(action_url)
        response.raise_for_status()  # Check for HTTP errors

        print(f"Motor action: {action_url}")
    except RequestException as e:
        print(f"Error: {e}")

# Main loop to take input from the user
while True:
    print("Press 1 to activate PUSH, or press 'q' to quit.")
    key = input()

    if key == '1':
        control_motor(push_url)
    elif key.lower() == 'q':
        break  # Exit the loop if 'q' is pressed
