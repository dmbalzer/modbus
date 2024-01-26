/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <modbus/modbus.h>

#if defined(_WIN32)
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#endif

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include <pthread.h>
#include <time.h>

#define NB_CONNECTION 5

uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
int master_socket;
int rc;
/* Maximum file descriptor number */
int fdmax;
fd_set refset;
fd_set rdset;

static char ip_addr[32];
static int port = 502;
static int nb_bits;
static int nb_input_bits;
static int nb_registers;
static int nb_input_registers;
static modbus_t *ctx = NULL;
static modbus_mapping_t *mb_mapping;
static int server_socket = -1;

static int init_lua_config(void);
static int load_lua_defaults(void);
static void close_sigint(int dummy);
static int init_modbus(void);
static void* run_modbus_thread(void* arg);
static void* run_lua_thread(void* arg);

int main(void)
{
    if ( init_lua_config() != 0 ) return -1;
    if ( init_modbus() != 0 ) return -1;
    if ( load_lua_defaults() != 0 ) return -1;
    pthread_t t1;
    pthread_t t2;
    pthread_create(&t1, NULL, run_modbus_thread, NULL);
    pthread_create(&t2, NULL, run_lua_thread, NULL);
    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    return 0;
}

static int init_lua_config(void)
{
    lua_State* L = luaL_newstate();
    luaL_openlibs(L);
    if ( luaL_dofile(L, "scripts/config.lua") != LUA_OK ) {
        fprintf(stderr, "Error:%s\n", lua_tostring(L, -1));
        lua_close(L);
        return -1;
    }
    lua_getfield(L, 1, "ip_addr");
    if ( lua_type(L, -1) != LUA_TSTRING ) {
        fprintf(stderr, "lua ip_address error.\n");
        lua_close(L);
        return -1;
    }
    sprintf(ip_addr, lua_tostring(L, -1));
    lua_pop(L, 1);

    lua_getfield(L, 1, "port");
    if ( lua_type(L, -1) != LUA_TNUMBER ) {
        fprintf(stderr, "lua port error.\n");
        lua_close(L);
        return -1;
    }
    port = lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, 1, "nb_bits");
    if ( lua_type(L, -1) != LUA_TNUMBER ) {
        fprintf(stderr, "lua nb_bits error.\n");
        lua_close(L);
        return -1;
    }
    lua_getfield(L, 1, "nb_input_bits");
    if ( lua_type(L, -1) != LUA_TNUMBER ) {
        fprintf(stderr, "lua nb_input_bits error.\n");
        lua_close(L);
        return -1;
    }
    lua_getfield(L, 1, "nb_registers");
    if ( lua_type(L, -1) != LUA_TNUMBER ) {
        fprintf(stderr, "lua nb_registers error.\n");
        lua_close(L);
        return -1;
    }
    lua_getfield(L, 1, "nb_input_registers");
    if ( lua_type(L, -1) != LUA_TNUMBER ) {
        fprintf(stderr, "lua nb_input_registers error.\n");
        lua_close(L);
        return -1;
    }

    nb_bits = lua_tointeger(L, -4);
    nb_input_bits = lua_tointeger(L, -3);;
    nb_registers = lua_tointeger(L, -2);;
    nb_input_registers = lua_tointeger(L, -1);;

    lua_close(L);
    return 0;
}

static int load_lua_defaults(void)
{
    lua_State* L = luaL_newstate();
    luaL_openlibs(L);
    if ( luaL_dofile(L, "scripts/config.lua") != LUA_OK ) {
        fprintf(stderr, "Error:%s\n", lua_tostring(L, -1));
        lua_close(L);
        return -1;
    }
    
    lua_getfield(L, 1, "def_bits");
    if ( lua_type(L, -1) != LUA_TTABLE ) {
        fprintf(stderr, "lua def_bits table error.\n");
        lua_close(L);
        return -1;
    }
    for ( int i = 0; i < nb_bits; i++ )
    {
        lua_geti(L, -1, i + 1);
        mb_mapping->tab_bits[i] = luaL_checkinteger(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    lua_getfield(L, 1, "def_input_bits");
    if ( lua_type(L, -1) != LUA_TTABLE ) {
        fprintf(stderr, "lua def_input_bits table error.\n");
        lua_close(L);
        return -1;
    }
    for ( int i = 0; i < nb_input_bits; i++ )
    {
        lua_geti(L, -1, i + 1);
        mb_mapping->tab_input_bits[i] = luaL_checkinteger(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    lua_getfield(L, 1, "def_registers");
    if ( lua_type(L, -1) != LUA_TTABLE ) {
        fprintf(stderr, "lua def_registers table error.\n");
        lua_close(L);
        return -1;
    }
    for ( int i = 0; i < nb_registers; i++ )
    {
        lua_geti(L, -1, i + 1);
        mb_mapping->tab_registers[i] = (uint16_t)luaL_checkinteger(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    lua_getfield(L, 1, "def_input_registers");
    if ( lua_type(L, -1) != LUA_TTABLE ) {
        fprintf(stderr, "lua def_input_registers table error.\n");
        lua_close(L);
        return -1;
    }
    for ( int i = 0; i < nb_input_registers; i++ )
    {
        lua_geti(L, -1, i + 1);
        mb_mapping->tab_input_registers[i] = (uint16_t)luaL_checkinteger(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    lua_close(L);
    return 0;
}

static void close_sigint(int dummy)
{
    if (server_socket != -1) {
        close(server_socket);
    }
    modbus_free(ctx);
    modbus_mapping_free(mb_mapping);

    exit(dummy);
}

static int init_modbus(void)
{
    ctx = modbus_new_tcp(ip_addr, port);

    mb_mapping =
        modbus_mapping_new(nb_bits, nb_input_bits, nb_registers, nb_input_registers);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);
    if (server_socket == -1) {
        fprintf(stderr, "Unable to listen TCP connection\n");
        modbus_free(ctx);
        return -1;
    }

    signal(SIGINT, close_sigint);

    /* Clear the reference set of socket */
    FD_ZERO(&refset);
    /* Add the server socket */
    FD_SET(server_socket, &refset);

    /* Keep track of the max file descriptor */
    fdmax = server_socket;
    return 0;
}

static void* run_modbus_thread(void* arg)
{
    for (;;) {
        // printf("*******************************top of main for loop*******************************\n");
        rdset = refset;
        if (select(fdmax + 1, &rdset, NULL, NULL, NULL) == -1) {
            perror("Server select() failure.");
            close_sigint(1);
        }

        /* Run through the existing connections looking for data to be
         * read */
        for (master_socket = 0; master_socket <= fdmax; master_socket++) {
            // printf("*******************************top of socket loop*******************************\n");
            if (!FD_ISSET(master_socket, &rdset)) {
                continue;
            }

            if (master_socket == server_socket) {
                /* A client is asking a new connection */
                socklen_t addrlen;
                struct sockaddr_in clientaddr;
                int newfd;

                /* Handle new connections */
                addrlen = sizeof(clientaddr);
                memset(&clientaddr, 0, sizeof(clientaddr));
                newfd = accept(server_socket, (struct sockaddr *) &clientaddr, &addrlen);
                if (newfd == -1) {
                    perror("Server accept() error");
                } else {
                    FD_SET(newfd, &refset);

                    if (newfd > fdmax) {
                        /* Keep track of the maximum */
                        fdmax = newfd;
                    }
                    printf("New connection from %s:%d on socket %d\n",
                           inet_ntoa(clientaddr.sin_addr),
                           clientaddr.sin_port,
                           newfd);
                }
            } else {
                modbus_set_socket(ctx, master_socket);
                // printf("**********************receiving*********************************\n");
                rc = modbus_receive(ctx, query);
                if (rc > 0) {
                    // printf("**********************replying*********************************\n");
                    modbus_reply(ctx, query, rc, mb_mapping);
                } else if (rc == -1) {
                    /* This example server in ended on connection closing or
                     * any errors. */
                    printf("Connection closed on socket %d\n", master_socket);
                    close(master_socket);

                    /* Remove from reference set */
                    FD_CLR(master_socket, &refset);

                    if (master_socket == fdmax) {
                        fdmax--;
                    }
                }
            }
        }
    }
}

static void* run_lua_thread(void* arg)
{
    for (;;)
    {
        sleep(5);
        lua_State* L = luaL_newstate();
        luaL_dofile(L, "scripts/script.lua");
        uint16_t val = (uint16_t)lua_tonumber(L, -1);
        mb_mapping->tab_input_registers[0] = val;
        lua_close(L);
    }
}
