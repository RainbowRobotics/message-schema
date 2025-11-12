from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)

from .deploy_module import (
    DeployService,
)

deploy_socket_router = RbSocketIORouter()
deploy_service = DeployService()


@deploy_socket_router.on("{sw_name}/{ip}/progress")
async def on_deploy_progress(data, sw_name: str, ip: str):
    dict_data = t_to_dict(data)
    dict_data["ip"] = ip
    dict_data["sw_name"] = sw_name

    res = await deploy_service.deploy_progress(request=dict_data)
    return to_json(res)
