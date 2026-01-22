const API_URL = "http://127.0.0.1:8000";

function loadStatus() {
  fetch(`${API_URL}/status`)
    .then(res => res.json())
    .then(data => {
      document.getElementById("name").innerText = data.name;
      document.getElementById("status").innerText = data.status;
      document.getElementById("mode").innerText = data.mode;
      document.getElementById("battery").innerText = data.battery;
      document.getElementById("speed").innerText = data.speed;
    });
}

function sendCommand(command) {
  fetch(`${API_URL}/control/${command}`, {
    method: "POST"
  })
  .then(() => loadStatus());
}

loadStatus();
setInterval(loadStatus, 1000);