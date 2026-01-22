from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dummy_data import robot_status

app = FastAPI()

# Agar frontend bisa akses backend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def root():
    return {"message": "Backend robot aktif"}

@app.get("/status")
def get_status():
    return robot_status

@app.post("/control/{command}")
def control_robot(command: str):
    if command == "start":
        robot_status["status"] = "RUNNING"
    elif command == "stop":
        robot_status["status"] = "STOPPED"
        robot_status["speed"] = 0.0
    elif command == "forward":
        robot_status["speed"] = 0.5
    elif command == "backward":
        robot_status["speed"] = -0.5

    return {
        "command": command,
        "robot_status": robot_status
    }
