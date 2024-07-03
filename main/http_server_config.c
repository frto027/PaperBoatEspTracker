#include "http_server_config.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "http_parser.h"
#include "ir_sensor.h"

ir_sensor_config_t * display_sensor;


esp_err_t data_get_handler(httpd_req_t *req)
{
    /* 发送回简单的响应数据包 */
    char response[256] = "{}";

    // sprintf(response, "{\"div_scale\":%d,\"sensor\":[[%d,%d],[%d,%d]]}", 
    //     TIMER_TICK_PER_US,
    //     display_sensor->angle_main_hori_us,
    //     display_sensor->angle_main_vert_us,
    //     display_sensor->angle_sub_hori_us ,
    //     display_sensor->angle_sub_vert_us
    // );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, 
    #include "../html/index.html.inl"
    , HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t uri_get_data = {
    .uri      = "/data",
    .method   = HTTP_GET,
    .handler  = data_get_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_get_index = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_get_handler,
    .user_ctx = NULL
};

//Function for starting the webserver
httpd_handle_t start_webserver(void)
{
     // Generate default configuration
     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
     config.core_id = 1;
     // Empty handle to http_server
     httpd_handle_t server = NULL;

     // Start the httpd server
     if (httpd_start(&server, &config) == ESP_OK) {
         // Register URI handlers
         httpd_register_uri_handler(server, &uri_get_data);
         httpd_register_uri_handler(server, &uri_get_index);
     }
     // If server failed to start, handle will be NULL
     return server;
}


void init_server_for_sensor(ir_sensor_config_t * sensor){
    display_sensor = sensor;
    start_webserver();
}