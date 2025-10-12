#!/bin/bash

###################
# 유�리티 함수들 #
###################

# 컬러 출력 함수
function print_string(){
  local RED='\033[0;31m'
  local GREEN='\033[0;32m'
  local YELLOW='\033[1;33m'
  local NC='\033[0m'

  case "$1" in
    "error") echo -e "${RED}${2}${NC}" ;;
    "success") echo -e "${GREEN}${2}${NC}" ;;
    "warning") echo -e "${YELLOW}${2}${NC}" ;;
    "info") echo -e "${NC}${2}${NC}" ;;
  esac
}

# 서비스 이름 입력 받는 함수
function get_service_name() {
    local app_names=()
    
    # print_string 출력을 /dev/tty로 강제 지정
    print_string "info" "배포할 서비스 이름을 입력해주세요 (여러 서비스는 쉼표로 구분, 전체는 all):" > /dev/tty

    # 한 줄만 읽도록 수정
    read -e input < /dev/tty

    if [ -z "${input}" ]; then
        print_string "error" "서비스 이름을 입력해주세요." > /dev/tty
        get_service_name
    fi
    
    # 입력값이 'all'인 경우
    if [ "${input}" = "all" ]; then
        for dir in backend/services/*/; do
            if [ -d "${dir}" ]; then
                # 디렉토리 경로에서 앱 이름만 추출
                local app_name=$(basename "$dir")
                # 중복 체크 후 추가
                if [[ ! " ${app_names[@]} " =~ " ${app_name} " ]]; then
                    app_names+=("${app_name}")
                fi
            fi
        done
    else
        # 기존 로직: 쉼표로 구분된 입력을 배열로 변환
        IFS=',' read -ra input_array <<< "$input"
        
        for app_name in "${input_array[@]}"; do
            # 앱 이름에서 공백 제거
            app_name=$(echo "${app_name}" | xargs)
            if [ -d "backend/services/${app_name}" ] && [[ ! " ${app_names[@]} " =~ " ${app_name} " ]]; then
                app_names+=("${app_name}")
            else
                print_string "warning" "'backend/services/${app_name}' 디렉토리가 존재하지 않습니다." > /dev/tty
            fi
        done
    fi
    
    # 배열을 구분자로 구분하여 문자열로 변환하여 반환
    local names_str=$(IFS=','; echo "${app_names[*]}")
    
    echo "$names_str"
}

# hotfix 여부를 선택하는 함수
function ask_hotfix() {
    local hotfix_response="no"  # 기본값을 "no"로 설정

    print_string "info" "이 배포가 hotfix 인가요? (y나 yes면 핫픽스, 아니면 Enter):" > /dev/tty

    read -r input < /dev/tty
    input=$(echo "$input" | tr '[:upper:]' '[:lower:]')  # 입력을 소문자로 변환

    if [[ "$input" == "y" || "$input" == "yes" ]]; then
        hotfix_response="yes"
    fi

    echo "${hotfix_response}"
}

# Git tag 작업 수행
function git_tag_work() {
    local new_version=$1
    local tag_version=$2
    local release_message=$3

    local bin_files=($(find backend/services -type f -name "*.bin"))

    if [ ${#bin_files[@]} -gt 0 ]; then
        print_string "info" "🔹 backend/services 내 .bin 파일들을 임시 add 중..."
        for bf in "${bin_files[@]}"; do
            git add -f "$bf" || { print_string "error" "Git add ${bf} 실패"; return 1; }
        done
    fi

    git commit --allow-empty -m "chore: Release ${new_version}" || {
        print_string "error" "Git commit 실패"
        return 1
    }

    git tag -a "${tag_version}" -m "${release_message}" || { print_string "error" "Git tag 실패"; return 1; }
    git push origin "${tag_version}" || { print_string "error" "Git tag push 실패"; return 1; }

    git tag -d "${tag_version}" || { print_string "error" "Git tag 삭제 실패"; return 1; }

    git reset --mixed HEAD~1 || { print_string "error" "Git reset 실패"; return 1; }

    if [ ${#bin_files[@]} -gt 0 ]; then
        print_string "info" "🧹 .bin 파일 ignore 상태 복구 중..."
        for bf in "${bin_files[@]}"; do
            git update-index --assume-unchanged "$bf" || true
        done
    fi

    print_string "success" "✅ 태그 생성 및 cleanup 완료"
    return 0
}


# 빌드 프로세스
function build_project() {
    local -n app_names=$1

    echo "app_names: ${app_names[@]}"

    print_string "info" "프로젝트 빌드 중..."
    
    local build_filters=""
    
    for app in "${app_names[@]}"; do
        app=$(echo "${app}" | tr [:upper:] [:lower:])
        build_filters+="${app} "
    done
    
    make backend.build SERVICE=${build_filters} || { 
        print_string "error" "빌드 실패"
        return 1
    }
    print_string "success" "패키지 설치 및 빌드 완료"
    return 0
}

# 버전 타입 선택 함수
function ask_version_type() {
    local version_type="patch"

    print_string "info" "배포할 버전 타입을 선택해주세요 (major/minor/patch, 기본값: patch):" > /dev/tty

    read -r input < /dev/tty
    input=$(echo "${input}" | tr '[:upper:]' '[:lower:]')

    if [[ "${input}" == "major" || "${input}" == "minor" || "${input}" == "patch" ]]; then
        version_type="${input}"
    fi

    echo "${version_type}"
}

# GitHub Release 생성 함수
function create_github_release() {
    local tag_version=$1
    local release_message=$2

    local app_name=$(echo "${tag_version}" | cut -d'/' -f2)

    # Release 생성
    gh release create "${tag_version}" \
        --title "Release ${tag_version}" \
        --notes "${release_message}" \
        --repo "$(git config --get remote.origin.url | sed 's/.*github.com[:/]//' | sed 's/\.git$//')" || {
            print_string "error" "GitHub Release 생성 실패"
            return 1
        }
    
    print_string "success" "[${app_name}] GitHub Release가 생성되었습니다: ${tag_version}"

    return 0
}

# 작업 내역 입력 받는 함수
function get_release_message() {
    local branch=$1
    local version=$2

    local release_message=""
    
    # print_string 출력을 /dev/tty로 강제 지정
    print_string "info" "작업 내역을 입력해주세요 (필수, 여러 줄 입력. 입력 완료 시 Enter를 치고 Control+D를 입력하세요):" > /dev/tty
    
    # read -e를 사용하여 편집 가능한 입력 받기
    while IFS= read -e line; do
        release_message+="$line"$'\n'
    done < /dev/tty
    
    release_message_trimmed=$(echo "${release_message}" | tr -d '[:space:]')
    if [ -z "${release_message_trimmed}" ]; then
        print_string "error" "작업 내역이 비어있습니다. 다시 입력해주세요." > /dev/tty
        release_message=$(get_release_message)
    fi
    
    echo "${release_message}\n [Download build file](https://rainbow-deploy.s3.ap-northeast-2.amazonaws.com/robot-repeater-server/${branch}/${version}.zip)"
}

# Release 버전 조회 후 새로운 버전 생성
function get_app_version() {
    local version_type=$1

    git fetch --tags > /dev/null

    version_tag=$(git tag --list "release/*" --sort=-v:refname | head -n 1)

    app_version=${version_tag##*/}
    app_version=${app_version:-0.0.1}

    IFS='.' read -r major minor patch <<< "${app_version}"

    case "${version_type}" in
        "major")
            major=$((major + 1))
            minor=0
            patch=0
            ;;
        "minor")
            minor=$((minor + 1))
            patch=0
            ;;
        *)
            patch=$((patch + 1))
            ;;
    esac

    new_version="${major}.${minor}.${patch}"

    echo "${new_version}"
}

# 완료 메시지 출력
function print_completion_message() {
    local new_version=$1
    
    print_string "success" "=================================="

    print_string "success" "✨🎉 $new_version 배포 완료 🎉✨"

    print_string "success" "=================================="
}

#######################
# 메인 스크립트 실행 #
#######################

if [[ "${current_branch}" = "main" ]]; then
    # GitHub CLI가 설치되어 있는지 확인
    if ! command -v gh &> /dev/null; then
        print_string "error" "GitHub CLI가 설치되어 있지 않습니다."
        exit 1
    fi
    
    # GitHub CLI 로그인 상태 확인
    if ! gh auth status &> /dev/null; then
        print_string "error" "GitHub CLI에 로그인되어 있지 않습니다. 'gh auth login'으로 로그인해주세요."
        exit 1
    fi
fi

# 현재 브랜치 저장
last_git_work_status="normal"

timestamp=$(date +%Y%m%d%H%M%S)
current_branch=$(git rev-parse --abbrev-ref HEAD)

# 배포할 서비스 이름 입력 받기
# 메인 스크립트에서 배열로 변환하여 사용
read names_str <<< $(get_service_name)

# 공백 제거
names_str=$(echo "${names_str}" | xargs)

if [[ "${current_branch}" = "main" ]]; then
    # 메인 스크립트 실행 부분에 hotfix 여부 확인 추가
    is_hotfix=$(ask_hotfix)
    echo "Hotfix 여부: ${is_hotfix}"

    version_type=$(ask_version_type)
    echo "버전 타입: ${version_type}"

    release_message=$(get_release_message "${current_branch}" "${new_version}")
fi

# 문자열을 다시 배열로 변환
IFS=',' read -ra app_names <<< "${names_str}"
underscore_names_str="${names_str//,/__}"

# 빌드 실행
# build_project "${app_names}" || exit 1

new_version="${timestamp}"

if [[ "${current_branch}" = "main" ]]; then
    new_version=$(get_app_version $version_type)
fi

# 태그 버전 생성
if [ "${is_hotfix}" = "yes" ]; then
    tag_version="deploy_hotfix/${underscore_names_str}/${current_branch}/${new_version}"
else
    tag_version="deploy/${underscore_names_str}/${current_branch}/${new_version}"
fi

# 배포 메시지 생성
deploy_message="deploy: [App: (${names_str}), Version: ${new_version}] 배포"

if [[ "$current_branch" = "main" ]] && [[ "$last_git_work_status" = "normal" ]]; then
    release_tag_version="release/${underscore_names_str}/${new_version}"
    git_tag_work "${new_version}" "${release_tag_version}" "${release_message}" || last_git_work_status="bad"
    create_github_release "${release_tag_version}" "${release_message}" || last_git_work_status="bad"
fi

if [[ "$last_git_work_status" = "normal" ]]; then
    # Git 작업 실행
    git_tag_work "${new_version}" "${tag_version}" "${deploy_message}" || last_git_work_status="bad"
fi


if [[ "${last_git_work_status}" = "bad" ]]; then
    git tag -d $tag_version
    git push origin --delete $tag_version
fi

# 완료 메시지 출력
if [ "$last_git_work_status" = "normal" ]; then
    print_completion_message "${new_version}"
    exit 0
else
    exit 1
fi


# # 각 앱에 대해 반복 처리
# for i in "${!app_names[@]}"; do
#     app_name="${app_names[$i]}"
#     app_dir="${app_dirs[$i]}"

#     if [[ "$last_git_work_status" = "bad" ]]; then
#         break
#     fi
    
#     # 현재 버전 생성
#     new_version="${app_name}-${timestamp}"

#     if [[ "$current_branch" = "main" ]]; then
#         new_version=$(get_app_version $version_type $app_name)
#     fi

#     # 태그 버전 생성
#     if [ "$is_hotfix" = "yes" ]; then
#         tag_version="deploy_hotfix/${app_name}/${current_branch}/${new_version}"
#     else
#         tag_version="deploy/${app_name}/${current_branch}/${new_version}"
#     fi

#     # 배포 메시지 생성
#     deploy_message="deploy: [App: ${app_name}, Version: ${new_version}] release 배포"

#     if [[ "$current_branch" = "main" ]] && [[ "$last_git_work_status" = "normal" ]]; then
#         release_tag_version="release/${app_name}/${new_version}"
#         git_tag_work "$new_version" "$release_tag_version" "$app_dir" "$app_name" "$release_message" || last_git_work_status="bad"
#         create_github_release "$release_tag_version" "$release_message" || last_git_work_status="bad"
#     fi

#     if [[ "$last_git_work_status" = "normal" ]]; then
#         # Git 작업 실행
#         git_tag_work "$new_version" "$tag_version" "$app_dir" "$app_name" "$deploy_message" || last_git_work_status="bad"
#     fi


#     if [[ "$last_git_work_status" = "bad" ]]; then
#         git tag -d $tag_version
#         git push origin --delete $tag_version
#     else
#         if [[  "$current_branch" = "main" ]]; then
#             new_versions+=("$app_name: v$new_version")
#         else
#             new_versions+=("v $new_version")
#         fi
#     fi
# done

# # 완료 메시지 출력
# if [ "$last_git_work_status" = "normal" ]; then
#     print_completion_message "$new_versions"
#     exit 0
# else
#     exit 1
# fi