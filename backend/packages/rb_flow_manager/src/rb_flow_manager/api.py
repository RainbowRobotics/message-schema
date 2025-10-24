import requests


class FlowManagerAPI:
    def __init__(self, access_token: str | None = None):
        self.base_url = "http://localhost:3000/common"
        self.session = requests.Session()
        if access_token:
            self.session.headers.update({"Authorization": f"Bearer {access_token}"})

    def get_task_status(self, task_id: str):
        r = self.session.get(f"{self.base_url}/task/{task_id}/status")
        r.raise_for_status()
        return r.json().get("status")

    def set_task_status(self, task_id: str, status: str):
        r = self.session.post(f"{self.base_url}/task/{task_id}/status", json={"status": status})
        r.raise_for_status()
        return r.json().get("status")

    def load_checkpoint(self, robot_model: str):
        r = self.session.get(f"{self.base_url}/{robot_model}/program/status")
        r.raise_for_status()

        doc = r.json()

        return {
            "task_id": doc.get("task_id"),
            "node_path": doc.get("node_path"),
            "offset": doc.get("offset"),
            "status": doc.get("status"),
        }

    def save_checkpoint(
        self, task_id: str, program_id: str, node_path: list[str], offset: int, status: str
    ):
        r = self.session.post(
            f"{self.base_url}/task/{task_id}/status",
            json={
                "program_id": program_id,
                "node_path": node_path,
                "offset": offset,
                "status": status,
            },
        )
        r.raise_for_status()

        doc = r.json()

        return {
            "task_id": doc.get("task_id"),
            "node_path": doc.get("node_path"),
            "offset": doc.get("offset"),
            "status": doc.get("status"),
        }

    def healty_check(self, robot_model: str):
        try:
            self.get_program_status(robot_model)
        except Exception:
            return False
        return True
