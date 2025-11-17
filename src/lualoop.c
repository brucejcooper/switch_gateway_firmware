#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <hardware/timer.h>
#include <pico/time.h>
#include <string.h>
#include "modbus.h"
#include <hardware/flash.h>
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
#include <sys/types.h>
#include <dirent.h>
#include <hardware/watchdog.h>
#include <malloc.h>
#include "pico/util/queue.h"


#define MAX_LINE_LEN 256

static queue_t runnableTasksQueue;
static queue_t btnChangeQueue;

static lua_State *mainThread;


typedef struct {
    lua_State *thread;
    int nargs;
} resume_args_t;

typedef struct {
    int btnidx:8;
    bool pressed:1;
} btn_change_t;


void scheduleResume(lua_State *thread, int nargs) {
    resume_args_t args = {
        .thread = thread,
        .nargs = nargs
    };
    if (!queue_try_add(&runnableTasksQueue, &args)) {
        printf("Could not schedule coro for execution");
    }
}


/*
** Try to compile line on the stack as 'return <line>;'; on return, stack
** has either compiled chunk or original line (if compilation failed).
*/
static int addreturn (lua_State *L, char *line) {
    const char *retline = lua_pushfstring(L, "return %s;", line);
    int status = luaL_loadbuffer(L, retline, strlen(retline), "=stdin");
    if (status == LUA_OK) {
        lua_remove(L,-2); // Remove the modified line
    } else {
        lua_pop(L, 2);  /* pop result from 'luaL_loadbuffer' and modified line */
    }
    return status;
}

void dumpstack(lua_State *L, char *before) {
    printf("Stack %s\n", before);
    int top=lua_gettop(L);
    if (top == 0) {
        printf("Stack empty\n");
    }
    for (int i=1; i <= top; i++) {
        printf("%d\t%s\t", i, luaL_typename(L,i));
        switch (lua_type(L, i)) {
        case LUA_TNUMBER:
            printf("%g\n",lua_tonumber(L,i));
            break;
        case LUA_TSTRING:
            printf("%s\n",lua_tostring(L,i));
            break;
        case LUA_TBOOLEAN:
            printf("%s\n", (lua_toboolean(L, i) ? "true" : "false"));
            break;
        case LUA_TNIL:
            printf("%s\n", "nil");
            break;
        default:
            printf("%p\n",lua_topointer(L,i));
            break;
        }
    }
}

static int l_listdir (lua_State *L) {
    const char *path = luaL_checkstring(L, 1); 
    
    DIR *dir = opendir(path);
    if (dir == NULL) {
        luaL_error(L, "no such directory");
    }
    struct dirent *dirp;
    lua_newtable(L);
    int count = 1;
    while ((dirp = readdir(dir)) != NULL) {
        lua_pushnumber(L, count++);
        lua_pushstring(L, dirp->d_name);
        lua_settable(L, -3);
    }
    printf("done\n");
    closedir(dir);
    return 1;  
}



static int l_catFile(lua_State *L) {
    const char *path = luaL_checkstring(L, 1); 

    FILE *fp = fopen(path, "r");
    if (!fp) {
        luaL_error(L, "File does not exist");
    }
    int ch;
    while ((ch = fgetc(fp)) != EOF) {
        putchar(ch);
    }
    fclose(fp);
    printf("--END--\n");
    return 0;  
}

static int l_reboot(lua_State *L) {
    // Enable watchdog with 100 ms timeout.  This will give the REPL
    // enough time to reply before it yanks the carpet out.
    watchdog_enable(200, 0); 
    return 0;  
}

static void modbus_get_coils_complete(int res, uint8_t *buf, void *ctx) {
    lua_State *L = (lua_State *) ctx;
    if (L) {
        if (res > 0) {
            // We assume they were requested in sets of 8 - we have no other way to tell.
            int count = buf[2] * 8; 
            uint8_t *dptr = buf + 3;
            lua_createtable(L, count, 0);
            while (count) {
                lua_pushboolean(L, (*dptr) & 0x01);
                lua_settable(L, -2);
                if ((--count) % 8 == 0) {
                    dptr++;
                } else {
                    *dptr >>= 1;
                }
            }
        } else {
            lua_pushnil(L);
        }
        scheduleResume(L, 1);
    } else {
        // Ignore the response
        printf("Ignoring get coils response, as it was called from the main thread\n");
    }
}


static void modbus_set_coil_complete(int res, uint8_t *buf, void *ctx) {
    lua_State *L = (lua_State *) ctx;
    if (L) {
        if (res > 0) {
            lua_pushboolean(L, true);
        } else {
            lua_pushboolean(L, false);
        }
        scheduleResume(L, 1);
    }
}


static int l_modbus_set_coil(lua_State *L) {
    const int modbus_device = luaL_checkinteger(L, 1); 
    const int coil = luaL_checkinteger(L, 2); 
    luaL_checktype(L, 3, LUA_TBOOLEAN); 
    const bool val = lua_toboolean(L, 3);

    if (L == mainThread) {
        luaL_error(L, "Can not call modbus from non-coroutine");
        return 0;
    }

    modbus_set_coil(modbus_device, coil, val, modbus_set_coil_complete, L);
    return lua_yield(L, 0);
}


static void modbus_dali_complete(int res, uint8_t *buf, void *ctx) {
    lua_State *L = (lua_State *) ctx;
    if (L) {
        if (res >= 5) {
            lua_pushinteger(L, buf[2]);
        } else if (res <= -5) {
            // a negative response indicates an error - We return the negative value of the error code. 
            lua_pushinteger(L, -buf[2]);
        } else {
            lua_pushnil(L);
        }
        scheduleResume(L, 1);
    }
}

static int l_modbus_dali(lua_State *L) {
    const int modbus_device = luaL_checkinteger(L, 1); 
    const int cmd = luaL_checkinteger(L, 2); 
    const bool repeat = lua_toboolean(L, 3);
    if (L == mainThread) {
        luaL_error(L, "must call from coroutine");
        return 0;
    }
    modbus_dali(modbus_device, cmd, repeat ? 2 : 1, modbus_dali_complete, L);
    return lua_yield(L, 0);
}



static int l_modbus_get_coils(lua_State *L) {
    const int modbus_device = luaL_checkinteger(L, 1); 
    const int start = luaL_checkinteger(L, 2); 
    const int count = luaL_checkinteger(L, 3); 

    if (L == mainThread) {
        luaL_error(L, "Can not call modbus from non-coroutine");
        return 0;
    }
    modbus_get_coils(modbus_device, start, count, modbus_get_coils_complete, L);
    return lua_yield(L, 0);
}

uint32_t getTotalHeap(void) {
    extern char __StackLimit, __bss_end__;
    return &__StackLimit - &__bss_end__;
}

static int l_heapsize(lua_State *L) {
    lua_pushinteger(L, getTotalHeap());
    return 1;
}

static int l_heapfree(lua_State *L) {
    struct mallinfo m = mallinfo();
    lua_pushinteger(L,  getTotalHeap() - m.uordblks);
    return 1;
}


static void modbus_set_hr_complete(int res, uint8_t *buf, void *ctx) {
    lua_State *L = (lua_State *) ctx;
    if (L) {
        if (res > 0) {
            int device = buf[0];
            int address = buf[2] << 8 | buf[3];
            // modbus_get_holding_registers(device, address, 1, modbus_get_hr_complete, ctx);
            lua_pushboolean(L, true);
        } else {
            lua_pushboolean(L, false);
        }
        scheduleResume(L, 1);
    }

}

static void modbus_get_hrs_complete(int res, uint8_t *buf, void *ctx) {
    lua_State *L = (lua_State *) ctx;
    if (L) {
        if (res > 0) {
            int bytecount = buf[2];
            lua_createtable(L, bytecount, 0);
            for (int byteoffset = 3; byteoffset < 3+bytecount; byteoffset +=2) {
                uint16_t val = (buf[byteoffset] << 8) | buf[byteoffset+1];
                lua_pushinteger(L, byteoffset/2);
                lua_pushinteger(L, val);
                lua_settable(L, -3);
            }
        } else {
            // Instead of pushing a table, we push an error message. 
            lua_pushnil(L);
        }
        // Resume the coroutine.
        scheduleResume(L, 1);
    }
}


static int l_modbus_set_holding_regsister(lua_State *L) {
    const int modbus_device = luaL_checkinteger(L, 1); 
    const int coil = luaL_checkinteger(L, 2); 
    const int val = luaL_checkinteger(L, 3); 

    modbus_set_holding_register(modbus_device, coil, val, modbus_set_hr_complete, L);
    return lua_yield(L, 0);
}

static int l_modbus_get_holding_regsisters(lua_State *L) {
    const int modbus_device = luaL_checkinteger(L, 1); 
    const int start_addr = luaL_checkinteger(L, 2); 
    const int count = luaL_checkinteger(L, 3); 

    if (L == mainThread) {
        luaL_error(L, "Can not call modbus from non-coroutine");
        return 0;
    }

    modbus_get_holding_registers(modbus_device, start_addr, count, modbus_get_hrs_complete, L);
    return lua_yield(L, 0);
}


int64_t delay_cb(alarm_id_t id, void *data) {
    lua_State *thread = (lua_State *) data;
    scheduleResume(thread, 0);
    return 0;
}

static int l_delay(lua_State *L) {
    const int delayms = luaL_checkinteger(L, 1); 
    if (add_alarm_in_ms(delayms, delay_cb, L, true)< 0) {
        luaL_error(L, "Could not create alarm");
        return 0;
    }
    return lua_yield(L, 0);
}




static int maybe_mkdir(const char* path, mode_t mode)
{
    struct stat st;
    errno = 0;

    /* Try to make the directory */
    if (mkdir(path, mode) == 0)
        return 0;

    /* If it fails for any reason but EEXIST, fail */
    if (errno != EEXIST)
        return -1;

    /* Check if the existing path is a directory */
    if (stat(path, &st) != 0)
        return -1;

    /* If not, fail with ENOTDIR */
    if (!S_ISDIR(st.st_mode)) {
        errno = ENOTDIR;
        return -1;
    }

    errno = 0;
    return 0;
}

int mkdir_p(const char *path)
{
    /* Adapted from http://stackoverflow.com/a/2336245/119527 */
    char *_path = NULL;
    char *p; 
    int result = -1;
    mode_t mode = 0777;

    errno = 0;

    /* Copy string so it's mutable */
    _path = strdup(path);
    if (_path == NULL)
        goto out;

    /* Iterate the string */
    for (p = _path + 1; *p; p++) {
        if (*p == '/') {
            /* Temporarily truncate */
            *p = '\0';

            if (maybe_mkdir(_path, mode) != 0)
                goto out;

            *p = '/';
        }
    }   

    if (maybe_mkdir(_path, mode) != 0)
        goto out;

    result = 0;

out:
    free(_path);
    return result;
}


void cleanup_thread(lua_State *L, lua_State *thread) {
    // do some cleanup on the main stack - first, find the thread in our tasks table.

    lua_getglobal(L, "table");
    lua_getfield(L, -1, "remove");
    lua_getglobal(L, "tasks");
    lua_pushnil(L); // First key
    int found = 0;
    while ((found = lua_next(L, -2)) != 0) {
        lua_State *candidateThread = lua_tothread(L, -1);
        if (candidateThread == thread) {
            lua_pop(L,1);                          // stack = table, removefn, tasks, idx
            lua_call(L, 2, 0);
            lua_closethread(thread, NULL);
            break;
        } else {
            lua_pop(L,1); // Doesn't pop the index, because we use that as the argument for lua_next
        }
    }
    if (!found) {
        printf(">>ERROR: Did not find task to remove when coro completed\n");
    }
    // Clean out whatever we put on the stack.
    lua_settop(L, 0);
}

static int l_panic(lua_State *L) {
    dumpstack(L, "PANIC");
    printf("PANIC: %s. resetting device\n", lua_tostring(L, -1));
    // Schedule a reboot after 200ms
    watchdog_enable(200, 0); 
    while (true) {
        ;
    }
    return 0;
}

static void dump_table(lua_State *L, int idx) {
    bool first = true;
    printf(">{\n");
    lua_pushnil(L);
    while (lua_next(L, -2) != 0) {
        // Lua_tostring can modify the stack, and we need to keep the key in its original form.
        // The value entry we can modify.
        lua_pushvalue(L, -2);
        printf(">\t%s\t%s\n", lua_tostring(L, -1), lua_tostring(L, -2));
        lua_pop(L, 2); // The converted string and the value
    }
    printf(">]\n");
}


static void printResponse(lua_State *L, int idx, char prefix) {
    // if (lua_istable(L, idx)) {
    //     // See if it has a custom tostring
    //     int type = luaL_getmetafield(L, idx, "__tostring");
    //     bool hasNoToString = type != LUA_TFUNCTION;
    //     lua_pop(L, 1); // Get rid of the meta function we just popped.

    //     // if (hasNoToString) {
    //     //     dump_table(L,idx);
    //     //     return;
    //     // }
    // } 
    lua_getglobal(L, "tostring");
    lua_pushvalue(L, idx);
    int error = lua_pcall(L, 1, 1, 0);
    printf("%c %s\n", prefix, lua_tostring(L, -1)); // It could be an error, or not, but we can print either way.
    lua_pop(L, 1); // Get rid of the string at the top.
}

static void addThread(lua_State *L, int idx) {
    lua_getglobal(L, "table");
    lua_getfield(L, -1, "insert");
    lua_getglobal(L, "tasks");
    lua_pushvalue(L, idx);
    lua_call(L, 2, 0); // Insert the thread into the table.
    lua_pop(L, 1);
}

static int l_schedule(lua_State *L) {
    luaL_checktype(L, 1, LUA_TTHREAD);
    lua_State *thread = lua_tothread(L, 1);
    addThread(L, 1);

    int nargs = lua_gettop(L) -1;
    if (nargs > 0) {
        lua_xmove(L, thread, nargs);
    }
    scheduleResume(thread, nargs);
    return 0;
}

static void lua_schedule_cmd(char *cmd) {
    lua_State *L = mainThread;
    lua_settop(L, 0); // We want a clean stack for this.
    lua_State *thread = lua_newthread(L);
    addThread(L, 1);
    lua_pop(L,1); // Take the thread off the stack

    // Create a function in the thread context to execute the command(s)
    int error = addreturn(thread, cmd);
    if (error != LUA_OK) {
        error = luaL_loadbuffer(thread, cmd, strlen(cmd), "=stdin");
    }
    // printf("Scheduling coro for %s\n", buf);
    scheduleResume(thread, 0);
}

void lua_schedule_btn_change(int btnidx, bool isPressed) {
    btn_change_t change = {
        .btnidx = btnidx,
        .pressed = isPressed,
    };
    queue_try_add(&btnChangeQueue, &change);
}

void lua_init() {
    queue_init(&runnableTasksQueue, sizeof(resume_args_t), 100);
    queue_init(&btnChangeQueue, sizeof(btn_change_t), 100);
    if (mkdir_p("/lua") != 0) {
        printf("Could not create LUA directory\n");
    }

    lua_State *L = mainThread = luaL_newstate();
    hard_assert(L != NULL);

    luaL_openlibs(L);

    lua_pushcfunction(L, l_listdir);
    lua_setglobal(L, "listdir");
    
    lua_pushcfunction(L, l_reboot);
    lua_setglobal(L, "reboot");

    lua_pushcfunction(L, l_modbus_set_coil);
    lua_setglobal(L, "modbus_set_coil");

    lua_pushcfunction(L, l_modbus_get_coils);
    lua_setglobal(L, "modbus_get_coils");

    lua_pushcfunction(L, l_modbus_set_holding_regsister);
    lua_setglobal(L, "modbus_set_holding_register");

    lua_pushcfunction(L, l_modbus_get_holding_regsisters);
    lua_setglobal(L, "modbus_get_holding_registers");

    lua_pushcfunction(L, l_modbus_dali);
    lua_setglobal(L, "modbus_dali");


    lua_pushcfunction(L, l_catFile);
    lua_setglobal(L, "catfile");

    lua_pushcfunction(L, l_heapfree);
    lua_setglobal(L, "heapfree");
    lua_pushcfunction(L, l_heapsize);
    lua_setglobal(L, "heapsize");

    // "enhance" the coroutine module by adding the a schedule and delay functions
    lua_getglobal(L, "coroutine");
    lua_pushcfunction(L, l_schedule);
    lua_setfield(L, -2, "schedule");
    lua_pushcfunction(L, l_delay);
    lua_setfield(L, -2, "delay");
    lua_pop(L, 1);


    // Create a tasks global to store references to tasks as they are scheduled. 
    lua_newtable(L);
    lua_setglobal(L, "tasks");

    // Load and run our main script.
    // Set global package.path to be our main search path.
    lua_getglobal(L, "package");
    lua_pushstring(L, "/lua/?.lua");
    lua_setfield(L, -2, "path");
    lua_pop(L, 1);

    lua_atpanic(L, l_panic);

    printf("Running main.lua file\n");
    if (luaL_dofile(L, "/lua/main.lua") != LUA_OK) {
        printf("Error: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1); // Remove error message
    }

    printf("starting main LUA REPL\n");
}


void lua_poll() {
    static char buf[MAX_LINE_LEN];
    static char *ptr = buf;

    int ch;
    int error;
    resume_args_t resumeCmd;
    btn_change_t btnChange;
    int nextTaskId = 1;
    lua_State *L = mainThread;

    if (queue_try_remove(&btnChangeQueue, &btnChange)) {
        sprintf(buf, "buttons[%d]=%s", btnChange.btnidx, btnChange.pressed  ? "true": "false");
        lua_schedule_cmd(buf);
    }
    
    if (queue_try_remove(&runnableTasksQueue, &resumeCmd)) {
        // absolute_time_t t1 = get_absolute_time();
        // We have a thread to run
        int nres;
        // printf("Resuming %p with %d args\n", resumeCmd.thread, resumeCmd.nargs);
        int status = lua_status(resumeCmd.thread);
        // printf("Resuming thread %p, in status %d\n", resumeCmd.thread, status);
        if (true || status != LUA_YIELD) {
            int error = lua_resume(resumeCmd.thread, NULL, resumeCmd.nargs, &nres);
            // printf("Schedule response %d, with %d responses\n", error, nres);

            // Print any responses, be they errors or normal
            if (nres > 0) {
                // printf("Moving %d items from %p (stack size %d) to the main thread\n", nres, resumeCmd.thread, lua_gettop(resumeCmd.thread));
                lua_xmove(resumeCmd.thread, L, nres);
                // dumpstack(L, "Responses to print");
                char prefix = error == LUA_OK ? '>' : error == LUA_YIELD ? '.' : '!';
                while (lua_gettop(L)) {
                    printResponse(L, 1, prefix);
                    lua_remove(L, 1);
                }
            }

            if (error != LUA_YIELD && resumeCmd.thread != mainThread ) {
                cleanup_thread(L, resumeCmd.thread);
            }
        } else {
            printf("attempt to resume a thread in status %d\n", status);
        }
        lua_settop(L, 0);
        // absolute_time_t t2 = get_absolute_time();
        // printf("Run loop took %lld us\n", absolute_time_diff_us(t1,t2));

    } else if ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == 3) {
            printf("\r^C\n");
            ptr = buf;
        } else if (ch == 8) {
            if (ptr > buf) {
                putchar(ch);
                putchar(' ');
                putchar(8);
                ptr--;
            }
        } else if (ch == '\n') {
            putchar(ch);
            *ptr = '\0';
            lua_schedule_cmd(buf);
            ptr = buf;
        } else if (ptr-buf < MAX_LINE_LEN && ch != '\r') {
            putchar(ch);
            *ptr++ = ch;
        }
    }
}

