import requests

# Define the base URL and parameters
base_url = "http://pcr.bounceme.net/insert_temp.php"
params = {
    "temperature": 605.5,
    "id": 1,
    "coords": "40.7128,-74.0060"
}

# Send the GET request
response = requests.get(base_url, params=params)

# Check the response status
if response.status_code == 200:
    print("Request successful")
    print(response.text)
else:
    print(f"Request failed with status code: {response.status_code}")
