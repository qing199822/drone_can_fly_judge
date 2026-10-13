#!/usr/bin/env bash

set -Eeuo pipefail

MODEL="egm96-5"
ARCHIVE_NAME="${MODEL}.tar.bz2"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CACHE_DIR="${GEOGRAPHICLIB_CACHE_DIR:-${SCRIPT_DIR}/geographiclib}"
ARCHIVE="${CACHE_DIR}/${ARCHIVE_NAME}"
INSTALL_ROOT="${GEOGRAPHICLIB_INSTALL_ROOT:-/usr/share/GeographicLib}"
PROJECT_PATH="project/geographiclib/geoids-distrib/${ARCHIVE_NAME}"

# You can override all download nodes, for example:
# GEOGRAPHICLIB_URL=https://your-mirror.example/egm96-5.tar.bz2 ./install_egm96_5.sh
if [[ -n "${GEOGRAPHICLIB_URL:-}" ]]; then
    URLS=("${GEOGRAPHICLIB_URL}")
else
    URLS=(
        "https://jaist.dl.sourceforge.net/${PROJECT_PATH}"
        "https://nchc.dl.sourceforge.net/${PROJECT_PATH}"
        "https://excellmedia.dl.sourceforge.net/${PROJECT_PATH}"
        "https://downloads.sourceforge.net/${PROJECT_PATH}?use_mirror=autoselect"
    )
fi

log() {
    printf '[egm96-5] %s\n' "$*"
}

die() {
    printf '[egm96-5] 错误: %s\n' "$*" >&2
    exit 1
}

run_as_root() {
    local parent_dir

    parent_dir="$(dirname -- "${INSTALL_ROOT}")"
    if (( EUID == 0 )) || [[ -w "${INSTALL_ROOT}" ]] || \
        [[ ! -e "${INSTALL_ROOT}" && -w "${parent_dir}" ]]; then
        "$@"
    elif command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        die "安装到 ${INSTALL_ROOT} 需要 root 权限，但系统中没有 sudo"
    fi
}

download_with_curl() {
    local url

    for url in "${URLS[@]}"; do
        log "尝试节点: ${url}"
        if curl --fail --location \
            --continue-at - \
            --connect-timeout 15 \
            --retry 5 \
            --retry-delay 2 \
            --speed-limit 10240 \
            --speed-time 30 \
            --output "${ARCHIVE}" \
            "${url}"; then
            return 0
        fi
        log "当前节点失败，保留已下载内容并切换节点"
    done

    return 1
}

download_with_wget() {
    local url

    for url in "${URLS[@]}"; do
        log "尝试节点: ${url}"
        if wget --continue --timeout=30 --tries=5 --output-document="${ARCHIVE}" "${url}"; then
            return 0
        fi
        log "当前节点失败，保留已下载内容并切换节点"
    done

    return 1
}

download_archive() {
    mkdir -p -- "${CACHE_DIR}"

    if [[ -s "${ARCHIVE}" ]]; then
        log "发现缓存文件，将从 $(du -h "${ARCHIVE}" | awk '{print $1}') 处继续下载"
    fi

    if command -v aria2c >/dev/null 2>&1; then
        log "使用 aria2 多线程下载"
        aria2c --continue=true --max-connection-per-server=8 --split=8 \
            --min-split-size=1M --connect-timeout=15 --timeout=30 \
            --max-tries=5 --retry-wait=2 \
            --dir="${CACHE_DIR}" --out="${ARCHIVE_NAME}" "${URLS[@]}"
    elif command -v curl >/dev/null 2>&1; then
        log "使用 curl 断点续传（安装 aria2 后可启用多线程）"
        download_with_curl
    elif command -v wget >/dev/null 2>&1; then
        log "使用 wget 断点续传（安装 aria2 后可启用多线程）"
        download_with_wget
    else
        die "未找到 aria2c、curl 或 wget"
    fi
}

verify_archive() {
    local entry
    local found_pgm=false
    local -a entries=()

    log "检查压缩包完整性"
    bzip2 --test -- "${ARCHIVE}" || return 1
    mapfile -t entries < <(tar -tjf "${ARCHIVE}")

    for entry in "${entries[@]}"; do
        case "${entry}" in
            geoids/|geoids/${MODEL}.wld|geoids/${MODEL}.pgm.aux.xml) ;;
            geoids/${MODEL}.pgm) found_pgm=true ;;
            *) die "压缩包中出现非预期路径: ${entry}" ;;
        esac
    done

    [[ "${found_pgm}" == true ]] || die "压缩包中缺少 geoids/${MODEL}.pgm"
}

install_archive() {
    local stage_dir

    stage_dir="$(mktemp -d -t geographiclib-egm96-5-XXXXXXXX)"
    trap 'rm -rf -- "${stage_dir:-}"' RETURN

    tar -xjf "${ARCHIVE}" -C "${stage_dir}"
    log "安装到 ${INSTALL_ROOT}/geoids"
    run_as_root mkdir -p -- "${INSTALL_ROOT}/geoids"
    run_as_root install -m 0644 -- \
        "${stage_dir}/geoids/${MODEL}.pgm" "${INSTALL_ROOT}/geoids/${MODEL}.pgm"

    if [[ -f "${stage_dir}/geoids/${MODEL}.wld" ]]; then
        run_as_root install -m 0644 -- \
            "${stage_dir}/geoids/${MODEL}.wld" "${INSTALL_ROOT}/geoids/${MODEL}.wld"
    fi

    if [[ -f "${stage_dir}/geoids/${MODEL}.pgm.aux.xml" ]]; then
        run_as_root install -m 0644 -- \
            "${stage_dir}/geoids/${MODEL}.pgm.aux.xml" \
            "${INSTALL_ROOT}/geoids/${MODEL}.pgm.aux.xml"
    fi
}

if (( $# > 1 )) || (( $# == 1 )) && [[ "$1" != "--force" ]]; then
    die "用法: $0 [--force]"
fi

if [[ -s "${INSTALL_ROOT}/geoids/${MODEL}.pgm" && "${1:-}" != "--force" ]]; then
    log "${INSTALL_ROOT}/geoids/${MODEL}.pgm 已存在，无需重复安装"
    log "如需覆盖，请执行: $0 --force"
    exit 0
fi

if ! verify_archive 2>/dev/null; then
    download_archive || die "所有下载节点均失败，可设置 GEOGRAPHICLIB_URL 后重试"
    verify_archive || die "下载文件不完整；缓存已保留，再次运行脚本会断点续传"
else
    log "缓存文件完整，跳过下载"
fi

install_archive

log "安装完成"
log "验证命令: echo '34.7466 113.6253' | GeoidEval -n ${MODEL}"
