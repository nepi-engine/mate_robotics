#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##
#######################################################################################################
# setup_new_driver.sh
#
# Turns this "<cat>_template" driver folder into your own NEPI driver: renames
# every <cat>_template file and rewrites the identifiers inside them (pkg_name /
# PKG_NAME, the node/discovery/driver class names + their NODE_DICT/DRIVER_DICT/
# DISCOVERY_DICT entries, node_launch_name, frame ids) so pkg_name<->PKG_NAME and
# yaml file_name/class_name<->the real files/classes stay in lock-step -- the
# rename the shipped drivers (idx_v4l2, lsx_deepsea_sealite, ...) follow.
#
# The device CATEGORY (idx/lsx/npx/ptx/rbx/svx) is auto-detected from this
# folder's name, which must be "<cat>_drivers" (what deploy_nepi_drivers.sh
# globs, and what the folder is called in the real nepi_drivers tree) or the
# legacy "<cat>_template". Only the FILES inside get renamed -- they are named
# "<cat>_template_*" regardless of the folder.
#
# Comments that reference the sibling templates (e.g. "see lsx_template/...")
# are left intact on purpose.
#
# USAGE
#   1. Copy the whole <cat>_drivers/ folder into your own repo/location
#      (this script rides along inside it), plus deploy_nepi_drivers.sh.
#   2. Edit the EDIT-THESE block below.
#   3. Run it:
#        ./setup_new_driver.sh            # rename in place (asks to confirm)
#        ./setup_new_driver.sh --dry-run  # just show what it would do
#        ./setup_new_driver.sh --yes      # skip the confirmation prompt
#
# After it runs, see GETTING_STARTED.md for TODO / syntax-check / deploy steps.
#######################################################################################################

# ============================================================================
#  EDIT THESE
# ============================================================================

# REQUIRED. snake_case device/driver name, WITHOUT the "<cat>_" prefix. This is
# what "template" becomes everywhere.
#   e.g. category idx + DRIVER_NAME "v4l2"           -> files idx_v4l2_*, pkg IDX_V4L2
#        category lsx + DRIVER_NAME "deepsea_sealite" -> files lsx_deepsea_sealite_*
#        category svx + DRIVER_NAME "servo_maestro"   -> files svx_servo_maestro_*
DRIVER_NAME="my_driver"

# Human-friendly name shown in the RUI (params display_name).
DISPLAY_NAME="My Driver"

# One-line description (params description).
DESCRIPTION="My NEPI driver"

# OPTIONAL. PascalCase base for the class names -> <base>Node / <base>Discovery /
# <base>Driver. Leave blank to auto-derive as <Cat><DriverName> (e.g. IdxV4l2).
# Set it to match the shipped short style if you prefer (e.g. "Sealite" ->
# SealiteNode / SealiteDiscovery).
CLASS_BASE=""

# OPTIONAL. Force the category if this folder is named neither "<cat>_drivers"
# nor "<cat>_template".
# One of: idx | lsx | npx | ptx | rbx | svx . Blank = auto-detect from the folder.
CATEGORY=""

# ============================================================================
#  ---- no need to edit below here ----
# ============================================================================

set -euo pipefail

VALID_CATS="idx lsx npx ptx rbx svx"

usage() {
  cat <<'EOF'
setup_new_driver.sh -- turn a template driver folder into your own driver.

Edit the EDIT-THESE block at the top (DRIVER_NAME, DISPLAY_NAME, DESCRIPTION,
optional CLASS_BASE / CATEGORY), then run:

  ./setup_new_driver.sh            rename in place (asks to confirm)
  ./setup_new_driver.sh --dry-run  show the plan, change nothing
  ./setup_new_driver.sh --yes      skip the confirmation prompt
  ./setup_new_driver.sh --help     this text

The category (idx/lsx/npx/ptx/rbx/svx) is auto-detected from the folder name,
which must be "<cat>_drivers" (or the legacy "<cat>_template"). The folder
itself is NOT renamed -- deploy_nepi_drivers.sh globs "<cat>_drivers"; only the
files inside are.
EOF
}

DRY_RUN=0
ASSUME_YES=0
for a in "$@"; do
  case "$a" in
    -n|--dry-run) DRY_RUN=1 ;;
    -y|--yes)     ASSUME_YES=1 ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "Unknown argument: $a"; echo "Try --help"; exit 1 ;;
  esac
done

# ---- Locate ourselves. This script lives INSIDE the <cat>_template folder. --
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)/$(basename "${BASH_SOURCE[0]}")"
DRV_DIR="$(dirname "$SELF")"
DRV_BASE="$(basename "$DRV_DIR")"

# ---- Determine the device category ----------------------------------------
CAT=""
if [ -n "${CATEGORY}" ]; then
  CAT="${CATEGORY}"
elif [[ "${DRV_BASE}" =~ ^([a-z]+)_(drivers|template)$ ]]; then
  CAT="${BASH_REMATCH[1]}"
fi
if [ -z "${CAT}" ]; then
  echo "ERROR: could not determine the device category."
  echo "       This folder is '${DRV_BASE}' -- expected '<cat>_drivers' (or the"
  echo "       legacy '<cat>_template'), with <cat> one of: ${VALID_CATS}."
  echo "       Either run from the template folder, or set CATEGORY."
  exit 1
fi
if ! [[ " ${VALID_CATS} " == *" ${CAT} "* ]]; then
  echo "ERROR: category '${CAT}' is not one of: ${VALID_CATS}" ; exit 1
fi

# ---- Validate the name ----------------------------------------------------
if [ -z "${DRIVER_NAME}" ] || [ "${DRIVER_NAME}" = "template" ]; then
  echo "ERROR: set DRIVER_NAME to your driver's name (not empty / not 'template')." ; exit 1
fi
if ! [[ "${DRIVER_NAME}" =~ ^[a-z][a-z0-9_]*$ ]]; then
  echo "ERROR: DRIVER_NAME must be snake_case: lowercase letters, digits and"
  echo "       underscores, starting with a letter (e.g. deepsea_sealite)."
  echo "       Got: '${DRIVER_NAME}'  (do NOT include the '${CAT}_' prefix)"
  exit 1
fi
# Guard against the user pre-pending the category by habit.
if [[ "${DRIVER_NAME}" == "${CAT}_"* ]]; then
  echo "ERROR: drop the '${CAT}_' prefix from DRIVER_NAME -- it is added for you."
  echo "       (use e.g. 'v4l2', not '${CAT}_v4l2')"
  exit 1
fi
if [ -n "${CLASS_BASE}" ] && ! [[ "${CLASS_BASE}" =~ ^[A-Za-z][A-Za-z0-9]*$ ]]; then
  echo "ERROR: CLASS_BASE must be PascalCase letters/digits only (e.g. Sealite)." ; exit 1
fi

# ---- Derive every name ----------------------------------------------------
CAT_PASCAL="${CAT^}"                 # idx -> Idx
CAT_UPPER="${CAT^^}"                 # idx -> IDX
NAME_UPPER="${DRIVER_NAME^^}"        # deepsea_sealite -> DEEPSEA_SEALITE

# PascalCase of the driver name: deepsea_sealite -> DeepseaSealite
NAME_PASCAL=""
IFS='_' read -ra _parts <<< "${DRIVER_NAME}"
for _p in "${_parts[@]}"; do NAME_PASCAL+="${_p^}"; done

CLASS_BASE="${CLASS_BASE:-${CAT_PASCAL}${NAME_PASCAL}}"   # e.g. IdxV4l2 or Sealite

OLD_FILE_BASE="${CAT}_template"      # idx_template
NEW_FILE_BASE="${CAT}_${DRIVER_NAME}"  # idx_v4l2
OLD_PKG="${CAT_UPPER}_TEMPLATE"      # IDX_TEMPLATE
NEW_PKG="${CAT_UPPER}_${NAME_UPPER}" # IDX_V4L2
OLD_CLASS_BASE="${CAT_PASCAL}Template"  # IdxTemplate

# ---- Show the plan --------------------------------------------------------
echo "======================================================================"
echo " NEPI ${CAT_UPPER} driver template -> new driver"
echo "======================================================================"
printf "  %-22s %s\n" "category (fixed)"  "${CAT}  (folder stays '${DRV_BASE}/')"
printf "  %-22s %s\n" "file prefix"       "${OLD_FILE_BASE}_*   -> ${NEW_FILE_BASE}_*"
printf "  %-22s %s\n" "pkg_name/PKG_NAME" "${OLD_PKG}       -> ${NEW_PKG}"
printf "  %-22s %s\n" "class names"       "${OLD_CLASS_BASE}{Node,Discovery,Driver} -> ${CLASS_BASE}{Node,Discovery,Driver}"
printf "  %-22s %s\n" "display_name"      "${DISPLAY_NAME}"
printf "  %-22s %s\n" "description"       "${DESCRIPTION}"
echo "----------------------------------------------------------------------"
echo " files to rename:"
shopt -s nullglob
_had_files=0
for f in "${DRV_DIR}/${OLD_FILE_BASE}"*; do
  _had_files=1
  bn="$(basename "$f")"
  printf "   %s\n" "${bn}  ->  ${NEW_FILE_BASE}${bn#${OLD_FILE_BASE}}"
done
if [ "${_had_files}" -eq 0 ]; then
  echo "   (none found matching ${OLD_FILE_BASE}* -- already renamed?)"
fi
echo "======================================================================"

if [ "${DRY_RUN}" -eq 1 ]; then
  echo "(dry run -- nothing changed)"
  exit 0
fi

if [ "${ASSUME_YES}" -ne 1 ]; then
  read -r -p "Proceed with the rename? [y/N] " reply
  case "$reply" in
    [yY]|[yY][eE][sS]) ;;
    *) echo "Aborted."; exit 0 ;;
  esac
fi

# ---- 1) Rewrite the category-prefixed identifiers in every text file ------
# Only the "<cat>_"/"<Cat>"/"<CAT>_" prefixed tokens are touched, so bare
# "template" words in comments -- and cross-references to the OTHER category
# templates (e.g. "see lsx_template/...") -- are deliberately left alone.
SED_ARGS=(
  -e "s/${OLD_CLASS_BASE}/${CLASS_BASE}/g"
  -e "s/${OLD_PKG}/${NEW_PKG}/g"
  -e "s/${OLD_FILE_BASE}/${NEW_FILE_BASE}/g"
)

echo "Rewriting identifiers in source files..."
while IFS= read -r -d '' f; do
  [ "$f" = "$SELF" ] && continue          # never edit this script
  grep -Iq . "$f" || continue             # skip binary files
  sed -i "${SED_ARGS[@]}" "$f"
done < <(find "$DRV_DIR" -type f -not -path '*/.git/*' -not -name '*.pyc' -print0)

# ---- 2) Set the free-text params fields precisely -------------------------
# (awk -v treats the values as literal text; leading whitespace is preserved.)
PARAMS_PATH="${DRV_DIR}/${OLD_FILE_BASE}_params.yaml"
if [ -f "${PARAMS_PATH}" ]; then
  awk -v dn="${DISPLAY_NAME}" -v de="${DESCRIPTION}" '
    match($0, /^[[:space:]]*/) { ws = substr($0, 1, RLENGTH) }
    $0 ~ /^[[:space:]]*display_name:/ { print ws "display_name: " dn; next }
    $0 ~ /^[[:space:]]*description:/  { print ws "description: " de; next }
    { print }
  ' "${PARAMS_PATH}" > "${PARAMS_PATH}.tmp" && mv "${PARAMS_PATH}.tmp" "${PARAMS_PATH}"
fi

# ---- 3) Rename the files (folder name stays <cat>_template) ----------------
echo "Renaming files..."
for f in "${DRV_DIR}/${OLD_FILE_BASE}"*; do
  bn="$(basename "$f")"
  newbn="${NEW_FILE_BASE}${bn#${OLD_FILE_BASE}}"
  [ "$bn" = "$newbn" ] && continue
  mv "$f" "${DRV_DIR}/${newbn}"
  echo "   ${bn} -> ${newbn}"
done

echo "======================================================================"
echo " Done. Your ${CAT_UPPER} driver files are renamed in:"
echo "   ${DRV_DIR}"
echo ""
echo " Notes:"
echo "   - The folder is still '${DRV_BASE}/' on purpose -- deploy_nepi_drivers.sh"
echo "     globs '<cat>_drivers', so keep it. Deploy with:  ./deploy_nepi_drivers.sh ${CAT}"
echo "   - Header comments still say 'TEMPLATE' -- harmless; they mark the lineage."
echo "   - Now fill the TODO: markers (see GETTING_STARTED.md), then syntax-check:"
echo "       cd \"${DRV_DIR}\""
echo "       python3 -c \"import ast,glob; [ast.parse(open(f).read()) for f in glob.glob('${NEW_FILE_BASE}_*.py')]; print('py OK')\""
echo "       python3 -c \"import yaml; print('yaml OK' if yaml.safe_load(open('${NEW_FILE_BASE}_params.yaml')) else 1)\""
echo ""
echo " You can delete this setup script now: rm \"${DRV_DIR}/$(basename "$SELF")\""
echo "======================================================================"
