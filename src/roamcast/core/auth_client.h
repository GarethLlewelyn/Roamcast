#pragma once

#include <Arduino.h>

typedef enum {
    RC_AUTH_NOT_STARTED,
    RC_AUTH_PROBING,
    RC_AUTH_AUTHENTICATED,
    RC_AUTH_NOT_REQUIRED,
    RC_AUTH_DEGRADED
} RcAuthState;

void        rc_auth_client_init(const char* device_id);
void        rc_auth_client_startup(const char* username, const char* password);
void        rc_auth_client_loop(const char* username, const char* password);
bool        rc_auth_client_is_authenticated();
bool        rc_auth_client_is_degraded();
RcAuthState rc_auth_client_get_state();
const char* rc_auth_client_get_token();
