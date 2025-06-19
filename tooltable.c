/*

  tooltable.c - file based tooltable, LinuxCNC format

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if TOOLTABLE_ENABLE == 1

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#include "grbl/vfs.h"
#include "grbl/strutils.h"
#include "grbl/gcode.h"
#include "grbl/stream.h"
#include "grbl/core_handlers.h"

#define N_POCKETS 25

static bool loaded = false;
static tool_pocket_t pockets[N_POCKETS];
static tool_id_t current_tool = 0;

static tool_select_ptr tool_select;
static on_tool_changed_ptr on_tool_changed;
static on_vfs_mount_ptr on_vfs_mount;
static on_report_options_ptr on_report_options;

static tool_pocket_t *get_pocket (tool_id_t tool_id)
{
    uint_fast16_t idx;
    tool_pocket_t *pocket = NULL;

    if(tool_id >= 0) for(idx = 0; idx < N_POCKETS; idx++) {
        if(pockets[idx].tool.tool_id == tool_id) {
            pocket = &pockets[idx];
            break;
        }
    }

    return pocket;
}

static tool_data_t *getTool (tool_id_t tool_id)
{
    tool_data_t *tool_data = NULL;

    tool_pocket_t *pocket;
    if((pocket = get_pocket(tool_id)) && (!settings.macro_atc_flags.random_toolchanger || pocket->pocket_id != -1))
        tool_data = &pocket->tool;

    return tool_data;
}

static tool_data_t *getToolByIdx (uint32_t idx)
{
    tool_pocket_t *pocket = idx < N_POCKETS ? &pockets[idx] : NULL;

    return pocket && pocket->tool.tool_id >= 0 ? &pocket->tool : NULL;
}

// Read selected tool data from persistent storage.
static pocket_id_t getPocket (tool_id_t tool_id)
{
    tool_pocket_t *pocket = get_pocket(tool_id);

    return pocket ? pocket->pocket_id : 0;
}

static bool writeTools (tool_data_t *tool_data)
{
    bool ok;
    uint_fast16_t idx, axis;

    for(idx = 0; idx < N_POCKETS; idx++) {
        if((ok = pockets[idx].tool.tool_id == tool_data->tool_id)) {
            memcpy(&pockets[idx].tool, tool_data, sizeof(tool_data_t));
            break;
        }
    }

    vfs_file_t *file;
    char buf[300], tmp[20];

    if((file = vfs_open("/linuxcnc/tooltable.tbl", "w"))) {

        for(idx = 1; idx < N_POCKETS; idx++) {
            if(pockets[idx].tool.tool_id >= 0) {
                sprintf(buf, "P%d T%d ", (uint16_t)pockets[idx].pocket_id, (uint16_t)pockets[idx].tool.tool_id);
                for(axis = 0; axis < N_AXIS; axis++) {
                    if(pockets[idx].tool.offset.values[axis] != 0.0f) {
                        sprintf(tmp, "%s%-.3f ", axis_letter[axis], pockets[idx].tool.offset.values[axis]);
                        strcat(buf, tmp);
                    }
                }
                strcat(buf, "\n");
                vfs_write(buf, strlen(buf), 1, file);
            }
        }

        vfs_close(file);
    }

    return ok;
}

static bool clearTools (void)
{
    uint_fast8_t idx;

    for(idx = 0; idx < N_POCKETS; idx++) {
        pockets[idx].tool.radius = 0.0f;
        memset(&pockets[idx].tool.offset, 0, sizeof(coord_data_t));
        if(!loaded) {
            pockets[idx].pocket_id = -1;
            pockets[idx].tool.tool_id = idx == 0 ? 0 : -1;
        }
    }

    return true;
}

static void loadTools (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    char c, buf[300];
    uint_fast8_t tools = 0, idx = 0, entry = 0, cc;
    vfs_file_t *file;
    status_code_t status = Status_GcodeUnusedWords;

    if((file = vfs_open("/linuxcnc/tooltable.tbl", "r"))) {

        while(vfs_read(&c, 1, 1, file) == 1) {

            if(c == ASCII_CR || c == ASCII_LF) {

                buf[idx] = '\0';

                if(!(*buf == '\0' || *buf == ';')) {

                   char *param = strtok(buf, " ");
                   tool_pocket_t pocket = { .pocket_id = -1, .tool.tool_id = -1 };

                   status = Status_OK;

                   while(param && status == Status_OK) {

                       cc = 1;

                       switch(CAPS(*param)) {

                           case 'T':
                               {
                                   uint32_t tool_id;
                                   if((status = read_uint(param, &cc, &tool_id)) == Status_OK)
                                       pocket.tool.tool_id = (tool_id_t)tool_id;
                               }
                               break;

                           case 'P':
                               {
                                   uint32_t pocket_id;
                                   if((status = read_uint(param, &cc, &pocket_id)) == Status_OK)
                                       pocket.pocket_id = (pocket_id_t)pocket_id;
                               }
                               break;

                           case 'X':
                               if(!read_float(param, &cc, &pocket.tool.offset.values[0]))
                                   status = Status_GcodeValueOutOfRange;
                               break;

                           case 'Y':
                               if(!read_float(param, &cc, &pocket.tool.offset.values[1]))
                                   status = Status_GcodeValueOutOfRange;
                               break;

                           case 'Z':
                               if(!read_float(param, &cc, &pocket.tool.offset.values[2]))
                                   status = Status_GcodeValueOutOfRange;
                               break;

                       }
                       param = strtok(NULL, " ");
                   }

                   if(status == Status_OK && pocket.tool.tool_id >= 0 && pocket.pocket_id >= 0) {

                       if(settings.macro_atc_flags.random_toolchanger) {
                           entry = pocket.pocket_id;
                       } else
                           entry++;

                       if(entry < N_POCKETS) {
                           tools++;
                           memcpy(&pockets[entry], &pocket, sizeof(tool_pocket_t));
                       }
                   }
                }

                idx = 0;

            } else if(idx < sizeof(buf))
                buf[idx++] = c;
        }

        loaded = tools > 0;

        vfs_close(file);
    }

    grbl.tool_table.n_tools = loaded ? N_POCKETS : 0;

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

static void onToolSelect (tool_data_t *tool, bool next)
{
    if(!next)
        current_tool = tool->tool_id;

    if(tool_select)
        tool_select(tool, next);
}

static void onToolChanged (tool_data_t *tool)
{
    if(settings.macro_atc_flags.random_toolchanger) {

        tool_pocket_t *from, *to;

        if((from = get_pocket(tool->tool_id)) && (to = get_pocket(current_tool))) {
            to->pocket_id = from->pocket_id;
            from->pocket_id = -1; // ??
            writeTools(&to->tool);
        }
    }

    current_tool = tool->tool_id;

    if(on_tool_changed)
        on_tool_changed(tool);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Tool table", "0.01");
}

void tooltable_init (void)
{
    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = loadTools;

    tool_select = hal.tool.select;
    hal.tool.select = onToolSelect;

    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = onToolChanged;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    grbl.tool_table.n_tools = N_POCKETS;
    grbl.tool_table.get_tool = getTool;
    grbl.tool_table.set_tool = writeTools;
    grbl.tool_table.get_tool_by_idx = getToolByIdx;
    grbl.tool_table.get_pocket = getPocket;
    grbl.tool_table.clear = clearTools;

    clearTools();

#if SDCARD_ENABLE
    sdcard_early_mount();
#endif
}

#endif // TOOLTABLE_ENABLE
