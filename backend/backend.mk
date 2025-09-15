PY ?= python3
ROOT_DIR := $(cd "$SCRIPT_DIR/../.." && pwd)
WORKDIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CONF_PORT ?= $(shell grep ^PORT= ${WORKDIR}/services/${SERVICE}/config.env | cut -d '=' -f2)

.PHONY: backend.sync
backend.sync: ## uv sync
	@cd ${WORKDIR} && uv sync --all-packages --frozen

.PHONY: backend.add
backend.add: ## uv add - SERVICE or PACKAGE 로 설치할 위치 지정 필수, DEPS 는 설치할 의존성 지정 필수 (ex. backend.add SERVICE=manipulate DEPS=fastapi)
	@bash -c '\
	if [ -n "$(PACKAGE)" ]; then \
		echo "📦 패키지 '$(PACKAGE)'에 러닝타임 의존성 추가 중: $(DEPS)"; \
		cd ${WORKDIR}/packages/${PACKAGE} && uv add ${IS_DEV:--dev} ${DEPS}; \
	elif [ -n "$(SERVICE)" ]; then \
		echo "📦 서비스 '$(SERVICE)'에 러닝타임 의존성 추가 중: $(DEPS)"; \
		cd ${WORKDIR}/services/${SERVICE} && uv add ${IS_DEV:--dev} ${DEPS}; \
	else \
		echo "📦 루트에 개발용 의존성 추가 중: ${DEPS}"; \
		cd ${WORKDIR} && uv add --dev ${DEPS}; \
	fi'

.PHONY: backend.lint
backend.lint: ## python ruff로 lint check 후 fix
	@cd ${WORKDIR} && \
	find . -path './.venv' -prune -o -name '*.py' -print0 \
	| xargs -0 uv run ruff check --fix --force-exclude

.PHONY: backend.local-dev
backend.local-dev: backend.lint backend.flatc
	@cd ${WORKDIR}/services/${SERVICE} && uv run uvicorn app.main:app --host 0.0.0.0 --port ${CONF_PORT} --reload --reload-dir ${WORKDIR} --reload-include '**/*.py'
	


.PHONY: backend.mac-dev
backend.mac-dev:  ## Backend 개발 환경 실행
	@bash ${WORKDIR}../api-gateway/generate-nginx-conf.sh
	@bash ${WORKDIR}../scripts/backend/generate-compose.sh
	@cd ${WORKDIR} && \
	uv run honcho -f ${WORKDIR}/Procfile.dev start

.PHONY: backend.dev
backend.dev:
	@bash ${ROOT_DIR}api-gateway/generate-nginx-conf.sh
	@bash ${ROOT_DIR}scripts/backend/generate-compose.sh
	@docker compose -f ${WORKDIR}/docker-compose.yml up

.PHONY: backend.build
backend.build: backend.flatc backend.lint ## 모든 Backend 서비스 또는 지정된 Backend 서비스 빌드
	@bash scripts/backend/build.sh

.PHONY: backend.build-only
backend.build-only:
	@bash scripts/backend/build.sh

.PHONY: backend.preview
backend.preview: ## Backend 운영 환경 실행 
	# @docker build -t rrs-nginx:latest api-gateway/
	@bash ${ROOT_DIR}api-gateway/generate-nginx-conf.sh
	@bash ${ROOT_DIR}scripts/backend/generate-compose.sh
	@docker compose -f ${WORKDIR}/docker-compose.preview.yml up

.PHONY: backend.flatc
backend.flatc: ## FlatBuffers 코드 생성
	-@find ${WORKDIR}packages/flat_buffers/src/flat_buffers -name "*.py" ! -name "__init__.py" -delete
	-@find ${WORKDIR}packages/flat_buffers/src/flat_buffers -name "*.py" ! -name "__init__.pyi" -delete
	@find ${WORKDIR}../schemas -name "*.fbs" \
		-exec flatc --python --gen-object-api --gen-all --python-typing --python-gen-numpy -o ${WORKDIR}/packages/flat_buffers/src/flat_buffers {} \;
	@$(PY) ${WORKDIR}/packages/flat_buffers/scripts/patch_imports.py ${WORKDIR}/packages/flat_buffers/src/flat_buffers