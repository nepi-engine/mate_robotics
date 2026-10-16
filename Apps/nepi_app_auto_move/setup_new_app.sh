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
# setup_new_app.sh
#
# Turns this "nepi_app_template" folder into your own NEPI app in one shot:
# renames every file and rewrites every "template"/"Template" identifier inside
# them (package name, node name, msg/class/connector/RUI-class names, script and
# param filenames, RUI element ids) so the result builds and runs under the new
# name -- following the exact naming conventions used by the shipped apps
# (nepi_app_nav_sim, nepi_app_pan_tilt_auto, ...).
#
# USAGE
#   1. Copy the whole nepi_app_template/ folder into your own repo/location
#      (this script rides along inside it), plus deploy_nepi_apps.sh next to it.
#   2. Edit the EDIT-THESE block below.
#   3. Run it:
#        ./setup_new_app.sh            # rename in place (asks to confirm)
#        ./setup_new_app.sh --dry-run  # just show what it would do
#        ./setup_new_app.sh --yes      # skip the confirmation prompt
#
# After it runs, see GETTING_STARTED.md for syntax-check / deploy / build steps.
#######################################################################################################

# ============================================================================
#  EDIT THESE
# ============================================================================

# REQUIRED. snake_case, the unique part of the name WITHOUT the "nepi_app_"
# prefix. This one value drives every derived name below.
#   e.g. "nav_sim"  ->  package nepi_app_nav_sim, node app_nav_sim,
#                       class NepiNavSimApp, msg NepiAppNavSimStatus, ...
APP_SUFFIX="auto_move"

# Human-friendly name shown in the RUI app menu.   e.g. "NavPose Sim"
DISPLAY_NAME="Auto Move App"

# One-line description (package.xml <description> + params display_name area).
DESCRIPTION="Auto move with obstacle avoidance application"

# RUI menu group the app is listed under. One of:
#   DEVICE | DATA | PROCESS | AUTOMATION | SYSTEM
GROUP_NAME="PROCESS"

# OPTIONAL. Short PascalCase name used ONLY for the RUI React class and RUI
# element ids, for when the full derived name would be unwieldy. This is the
# "shortname" some shipped apps use (e.g. pan_tilt_auto ships its RUI class as
# "NepiAppPTAuto"). Leave blank to auto-derive PascalCase from APP_SUFFIX
# (nav_sim -> NavSim). The msg type + package name always use the FULL name so
# the node and RUI still agree on the topic types.
SHORT_NAME=""

# ============================================================================
#  ---- no need to edit below here ----
# ============================================================================

set -euo pipefail

usage() {
  sed -n '11,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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

# ---- Locate ourselves. This script lives INSIDE the app folder. ------------
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)/$(basename "${BASH_SOURCE[0]}")"
APP_DIR="$(dirname "$SELF")"
PARENT_DIR="$(dirname "$APP_DIR")"

# ---- Validate inputs ------------------------------------------------------
if [ -z "${APP_SUFFIX}" ]; then
  echo "ERROR: APP_SUFFIX is empty. Edit the EDIT-THESE block first." ; exit 1
fi
if [ "${APP_SUFFIX}" = "template" ]; then
  echo "ERROR: APP_SUFFIX is still 'template'. Set it to your app's name." ; exit 1
fi
if ! [[ "${APP_SUFFIX}" =~ ^[a-z][a-z0-9_]*$ ]]; then
  echo "ERROR: APP_SUFFIX must be snake_case: lowercase letters, digits and"
  echo "       underscores, starting with a letter (e.g. nav_sim). Got: '${APP_SUFFIX}'"
  exit 1
fi
if [ -n "${SHORT_NAME}" ] && ! [[ "${SHORT_NAME}" =~ ^[A-Za-z][A-Za-z0-9]*$ ]]; then
  echo "ERROR: SHORT_NAME must be PascalCase letters/digits only (e.g. PTAuto)." ; exit 1
fi

# ---- Derive every name from APP_SUFFIX ------------------------------------
# PascalCase: nav_sim -> NavSim, pan_tilt_auto -> PanTiltAuto
PASCAL=""
IFS='_' read -ra _parts <<< "${APP_SUFFIX}"
for _p in "${_parts[@]}"; do PASCAL+="${_p^}"; done

RUI_PASCAL="${SHORT_NAME:-$PASCAL}"       # RUI class + element ids (may be short)
UPPER="${APP_SUFFIX^^}"                    # NAV_SIM  (used in log strings)

PKG_NAME="nepi_app_${APP_SUFFIX}"         # catkin pkg / msg module / folder
NODE_NAME="app_${APP_SUFFIX}"             # DEFAULT_NODE_NAME / appName
FILE_BASE="${APP_SUFFIX}_app"             # <base>_node.py / <base>_params.yaml
CONFIG_FILE="app_${APP_SUFFIX}.yaml"      # APP_DICT.config_file (informational)

MSG_CLASS="NepiApp${PASCAL}Status"        # full name, must match node<->rui
NODE_CLASS="Nepi${PASCAL}App"             # python node class
CONNECT_CLASS="ConnectApp${PASCAL}"       # python connector class
RUI_CLASS="NepiApp${RUI_PASCAL}"          # React class + file (may be short)
ID_PREFIX="App${RUI_PASCAL}"              # RUI element id prefix

# ---- Original (template) file paths, relative to APP_DIR -------------------
OLD_NODE="scripts/template_app_node.py"
OLD_PARAMS="params/template_app_params.yaml"
OLD_MSG="msg/NepiAppTemplateStatus.msg"
OLD_RUI="rui/NepiAppTemplate.js"
OLD_CONNECT="api/connect_app_template.py"

NEW_NODE="scripts/${FILE_BASE}_node.py"
NEW_PARAMS="params/${FILE_BASE}_params.yaml"
NEW_MSG="msg/${MSG_CLASS}.msg"
NEW_RUI="rui/${RUI_CLASS}.js"
NEW_CONNECT="api/connect_app_${APP_SUFFIX}.py"

# ---- Show the plan --------------------------------------------------------
echo "======================================================================"
echo " NEPI app template -> new app"
echo "======================================================================"
printf "  %-24s %s\n" "package (folder)"   "nepi_app_template          -> ${PKG_NAME}"
printf "  %-24s %s\n" "node name"          "app_template               -> ${NODE_NAME}"
printf "  %-24s %s\n" "node class"         "NepiTemplateApp            -> ${NODE_CLASS}"
printf "  %-24s %s\n" "connector class"    "ConnectAppTemplate         -> ${CONNECT_CLASS}"
printf "  %-24s %s\n" "status msg"         "NepiAppTemplateStatus      -> ${MSG_CLASS}"
printf "  %-24s %s\n" "RUI class"          "NepiAppTemplate            -> ${RUI_CLASS}"
printf "  %-24s %s\n" "RUI element ids"    "AppTemplate*               -> ${ID_PREFIX}*"
printf "  %-24s %s\n" "config file ref"    "app_template.yaml          -> ${CONFIG_FILE}"
printf "  %-24s %s\n" "display_name"       "${DISPLAY_NAME}"
printf "  %-24s %s\n" "description"        "${DESCRIPTION}"
printf "  %-24s %s\n" "group_name"         "${GROUP_NAME}"
echo "----------------------------------------------------------------------"
echo " file renames:"
printf "   %s\n" \
  "${OLD_NODE}    -> ${NEW_NODE}" \
  "${OLD_PARAMS}  -> ${NEW_PARAMS}" \
  "${OLD_MSG}     -> ${NEW_MSG}" \
  "${OLD_RUI}     -> ${NEW_RUI}" \
  "${OLD_CONNECT} -> ${NEW_CONNECT}" \
  "<folder>/nepi_app_template -> <folder>/${PKG_NAME}"
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

# ---- 1) Rewrite identifiers inside every text file ------------------------
# Order matters: more specific tokens first so a prefix never eats a longer
# token (e.g. NepiAppTemplateStatus before NepiAppTemplate before AppTemplate;
# nepi_app_template before app_template). The final bare "Template" catch-all
# tidies leftover comment text; free-text fields are set precisely afterwards.
SED_ARGS=(
  -e "s/NepiAppTemplateStatus/${MSG_CLASS}/g"
  -e "s/NepiAppTemplate/${RUI_CLASS}/g"
  -e "s/NepiTemplateApp/${NODE_CLASS}/g"
  -e "s/ConnectAppTemplate/${CONNECT_CLASS}/g"
  -e "s/AppTemplate/${ID_PREFIX}/g"
  -e "s/nepi_app_template/${PKG_NAME}/g"
  -e "s/template_app/${FILE_BASE}/g"
  -e "s/app_template/${NODE_NAME}/g"
  -e "s/TEMPLATE_APP/${UPPER}/g"
  -e "s/Template/${PASCAL}/g"
)

echo "Rewriting identifiers in source files..."
while IFS= read -r -d '' f; do
  [ "$f" = "$SELF" ] && continue          # never edit this script
  grep -Iq . "$f" || continue             # skip binary files
  sed -i "${SED_ARGS[@]}" "$f"
done < <(find "$APP_DIR" -type f \
           -not -path '*/.git/*' \
           -not -name '*.psd' -not -name '*.webp' -not -name '*.jpg' \
           -not -name '*.jpeg' -not -name '*.png' -not -name '*.pyc' -print0)

# ---- 2) Set the free-text config fields precisely -------------------------
# (awk -v treats the values as literal text -> safe for spaces/punctuation.)
PARAMS_PATH="${APP_DIR}/${OLD_PARAMS}"
if [ -f "${PARAMS_PATH}" ]; then
  awk -v dn="${DISPLAY_NAME}" -v de="${DESCRIPTION}" -v gn="${GROUP_NAME}" '
    /^[[:space:]]*display_name:/ { print "  display_name: " dn; next }
    /^[[:space:]]*description:/  { print "  description: " de; next }
    /^[[:space:]]*group_name:/   { print "  group_name: " gn; next }
    { print }
  ' "${PARAMS_PATH}" > "${PARAMS_PATH}.tmp" && mv "${PARAMS_PATH}.tmp" "${PARAMS_PATH}"
fi

PKG_XML="${APP_DIR}/package.xml"
if [ -f "${PKG_XML}" ]; then
  awk -v de="${DESCRIPTION}" '
    /<description>/ { print "  <description>" de "</description>"; next }
    { print }
  ' "${PKG_XML}" > "${PKG_XML}.tmp" && mv "${PKG_XML}.tmp" "${PKG_XML}"
fi

# ---- 3) Rename the inner files -------------------------------------------
echo "Renaming files..."
rename_one() {
  local from="${APP_DIR}/$1" to="${APP_DIR}/$2"
  if [ -f "$from" ]; then
    [ "$from" = "$to" ] && return 0
    mv "$from" "$to"
    echo "   $1 -> $2"
  fi
}
rename_one "${OLD_NODE}"    "${NEW_NODE}"
rename_one "${OLD_PARAMS}"  "${NEW_PARAMS}"
rename_one "${OLD_MSG}"     "${NEW_MSG}"
rename_one "${OLD_RUI}"     "${NEW_RUI}"
rename_one "${OLD_CONNECT}" "${NEW_CONNECT}"

# ---- 4) Rename the app folder itself (last) ------------------------------
NEW_APP_DIR="${PARENT_DIR}/${PKG_NAME}"
if [ "${APP_DIR}" != "${NEW_APP_DIR}" ]; then
  if [ -e "${NEW_APP_DIR}" ]; then
    echo "WARNING: ${NEW_APP_DIR} already exists -- leaving folder name unchanged."
    NEW_APP_DIR="${APP_DIR}"
  else
    mv "${APP_DIR}" "${NEW_APP_DIR}"
    echo "   <folder>/$(basename "${APP_DIR}") -> <folder>/${PKG_NAME}"
  fi
fi

echo "======================================================================"
echo " Done. Your app now lives at:"
echo "   ${NEW_APP_DIR}"
echo ""
echo " Next:"
echo "   - Fill in your real params/pubs/subs/logic (see GETTING_STARTED.md)."
echo "   - Syntax check:"
echo "       cd \"${NEW_APP_DIR}\""
echo "       python3 -c \"import ast; ast.parse(open('scripts/${FILE_BASE}_node.py').read())\""
echo "       python3 -c \"import yaml; assert 'APP_DICT' in yaml.safe_load(open('params/${FILE_BASE}_params.yaml'))\""
echo "   - Deploy with ./deploy_nepi_apps.sh, then rebuild the workspace + RUI."
echo ""
echo " You can delete this setup script now: rm \"${NEW_APP_DIR}/$(basename "$SELF")\""
echo "======================================================================"
