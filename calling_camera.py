import requests

def trigger_ip_camera_snapshot(ip, port, username, password, save_path='snapshot.jpg'):
    # Construct the URL for snapshot (this URL varies by camera model)
    snapshot_url = f"http://{ip}:{port}/snapshot.jpg"
    
    try:
        response = requests.get(snapshot_url, auth=(username, password), stream=True)
        if response.status_code == 200:
            with open(save_path, 'wb') as f:
                for chunk in response.iter_content(1024):
                    f.write(chunk)
            print(f"Snapshot saved to {save_path}")
        else:
            print(f"Failed to retrieve snapshot. Status code: {response.status_code}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Replace with your IP camera's details
    ip = '192.168.1.100'
    port = '1103'
    username = 'admin'
    password = 'password'
    trigger_ip_camera_snapshot(ip, port, username, 'ip_snapshot.jpg')
