# backend 초기 설정하기

## 1. python 설치

### 1-1. Windows:

#### [공식 설치 프로그램]

[Python.org 다운로드 페이지](https://www.python.org/downloads/windows/)에서 최신 Windows installer (exe) 다운로드.

실행 후 **"Add Python to PATH"** 옵션 꼭 체크.

#### [Chocolatey]

```bash
choco install python
```

#### [Scoop]

```bash
scoop install python
```

### 1-2. Linux (Ubuntu/Debian):

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv
```

### 1-3. macOS:

> macOS에 기본적으로 Python 2가 있을 수 있으니 최신은 직접 설치해야 함.

#### [공식 설치 프로그램]

[Python.org 다운로드 페이지](https://www.python.org/downloads/macos/)에서 .pkg 파일 다운로드 후 설치.

#### [Homebrew]

```bash
brew install python
```

## 2. pyenv 설치

### 2-1. Windows (PowerShell):

Windows에서는 pyenv-win 이란 걸 이용하여 pyenv를 설치할 수 있다고 한다.

아래의 깃허브 공식 홈페이지에 들어간다.

https://github.com/pyenv-win/pyenv-win

![pyenv-install-example1](https://velog.velcdn.com/images/pikamon/post/7a0dabef-72f7-4fde-bc4a-813af7621721/image.png)

스크롤을 내리다 보면 Installation 부분에 PowerShell을 이용하면 가장 쉽게 설치할 수 있다고 나와있다.

![pyenv-install-example2](https://velog.velcdn.com/images/pikamon/post/9633cb68-ba62-4aa2-9929-1aa1f1fb12c2/image.png)

PowerShell을 클릭하면 PowerShell을 이용한 설치 방법을 확인할 수 있다.

![pyenv-install-example2](https://velog.velcdn.com/images/pikamon/post/a8244a0d-0a39-4b61-9358-14881c8db26a/image.png)

PowerShell을 열고 아래 명령어를 입력한다.

```bash
Invoke-WebRequest -UseBasicParsing -Uri "https://raw.githubusercontent.com/pyenv-win/pyenv-win/master/pyenv-win/install-pyenv-win.ps1" -OutFile "./install-pyenv-win.ps1"; &"./install-pyenv-win.ps1"
```

그러면 아래와 같이 결과가 출력되며 pyenv가 설치되는 것을 볼 수 있다.

![pyenv-install-example2](https://velog.velcdn.com/images/pikamon/post/a11db283-7eb7-402f-bca9-9c4320ff5c26/image.png)

만약 아래와 같이 이 시스템에서 스크립트를 실행할 수 없으므로 install-pyenv-win.ps1 파일을 로드할 수 없습니다. 라는 에러가 발생한다면,

```
cannot be loaded because running scripts is disabled on this system
```

![pyenv-install-example2](https://velog.velcdn.com/images/pikamon/post/b639a23c-629e-4956-b5b5-0f550a98a4e1/image.png)

PowerShell을 관리자 권한으로 연 다음 아래 명령어를 한 번 실행한 후 재시도한다.

```bash
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine
```

그러면 아래와 같이 정상 설치되는 것을 볼 수 있다.

![pyenv-install-example2](https://velog.velcdn.com/images/pikamon/post/4289a383-1b69-450f-9269-ba5a1be93181/image.png)

정상 설치되었다면 터미널을 종료한 후 새로 열어준다.

~/.bashrc 최상단에 반드시 아래 두 줄을 추가:

```bash
export PYENV="$HOME/.pyenv/pyenv-win"
export PATH="$PYENV/shims:$PYENV/bin:$PATH"
```

### 2-2. Linux (Ubuntu/Debian):

```bash
sudo apt-get update
sudo apt-get install -y make build-essential libssl-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev curl git \
  libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```

그 다음:

```bash
curl https://pyenv.run | bash
```

~/.bashrc 또는 ~/.zshrc에 추가:

```bash
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
```

적용:

```bash
source ~/.bashrc
```

### 2-3. macOS

```bash
brew update
brew install pyenv
```

그리고 ~/.bashrc 또는 ~/.zshrc에 다음을 추가:

```bash
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
```

적용:

```bash
source ~/.bashrc   # 또는 ~/.zshrc
```

## 3. uv 설치

### 3-1. Windows (PowerShell):

```bash
irm https://astral.sh/uv/install.ps1 | iex
```

### 3-2. Linux/macOS:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

## 4. make 설치

### 3-1. Windows (PowerShell):

```bash
choco install make
```

### 3-2. Linux (Ubuntu/Debian 계열):

```bash
sudo apt-get update
sudo apt-get install -y make
```

### 3-3. macOS:

> macOS는 기본적으로 `make`명령어가 존재함 없으면 진행 바람

```bash
brew install make
```

## 4. Backend 프로젝트 초기화

위 사항을 모두 다 했다면 아래의 명령어를 실행하여 backend 프로젝트를 최종적으로 세팅을 마무리 한다.

```bash
bash script/backend/init-project.sh
```
