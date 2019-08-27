#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Function to start the file server */
esp_err_t start_file_server(const char *base_path);

#ifdef __cplusplus
} /* extern "C" */
#endif