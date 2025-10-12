from app.features.deploy.deploy_module import DeployService
from app.features.deploy.deploy_schema import DeployProgressSchemaPD
from fastapi import APIRouter

deploy_service = DeployService()
deploy_router = APIRouter(tags=["Deploy"])


@deploy_router.post(
    "{sw_name}/{ip}/progress",
    description="'deploy/{sw_name}/{ip}/progress' 의 토픽으로 socketio 구독 가능",
    response_model=DeployProgressSchemaPD,
)
async def deploy_progress(request: DeployProgressSchemaPD):
    return deploy_service.deploy_progress(request)
