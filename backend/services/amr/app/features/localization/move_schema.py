
# from pydantic import (
#     BaseModel,
#     Field,
# )


# class Request_Localization_InitPD(BaseModel):
#     """
#     [위치 초기화 (수동)]
#     - x: x 좌표
#     - y: y 좌표
#     - z: z 좌표
#     - rz: rz 좌표
#     """
#     x: float = Field(..., description="x 좌표")
#     y: float = Field(..., description="y 좌표")
#     z: float | None = Field(None, description="z 좌표")
#     rz: float = Field(..., description="rz 좌표")

# class ResponseLocalizationInitPD(BaseModel):
#     """
#     [위치 초기화 (수동) 응답]
#     - x: x 좌표
#     - y: y 좌표
#     - z: z 좌표
#     - rz: rz 좌표
#     - result: 결과
#     - message: 메시지
#     """
#     x: float = Field(..., description="x 좌표")
#     y: float = Field(..., description="y 좌표")
#     z: float | None = Field(None, description="z 좌표")
#     rz: float = Field(..., description="rz 좌표")
#     result: str = Field(..., description="결과")
#     message: str = Field(..., description="메시지")

# class RequestLocalizationRandomInitPD(BaseModel):
#     """
#     [위치 초기화 (랜덤)]
#     """

# class ResponseLocalizationRandomInitPD(BaseModel):
#     """
#     [위치 초기화 (랜덤) 응답]
#     - x: x 좌표
#     - y: y 좌표
#     - z: z 좌표
#     - rz: rz 좌표
#     - result: 결과
#     - message: 메시지
#     """
#     x: float = Field(..., description="x 좌표")
#     y: float = Field(..., description="y 좌표")
#     z: float | None = Field(None, description="z 좌표")
#     rz: float = Field(..., description="rz 좌표")
#     result: str = Field(..., description="결과")
#     message: str = Field(..., description="메시지")

# class RequestLocalizationAutoInitPD(BaseModel):
#     """
#     [위치 초기화 (자동)]
#     """

# class ResponseLocalizationAutoInitPD(BaseModel):
#     """
#     [위치 초기화 (자동) 응답]
#     - x: x 좌표
#     - y: y 좌표
#     - z: z 좌표
#     - rz: rz 좌표
#     - result: 결과
#     - message: 메시지
#     """
#     x: float = Field(..., description="x 좌표")
#     y: float = Field(..., description="y 좌표")
#     z: float | None = Field(None, description="z 좌표")
#     rz: float = Field(..., description="rz 좌표")
#     result: str = Field(..., description="결과")
#     message: str = Field(..., description="메시지")
