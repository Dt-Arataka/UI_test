#pragma once
#include <Arduino.h>

// 用函数指针做回调：简单、稳定、占用小
typedef void (*ims_remote_cb_t)();

class IMS_RemoteWeb {
public:
  bool beginAP(const char* ssid, const char* pass);
  void onStart(ims_remote_cb_t cb) { cb_start_ = cb; }
  void onPause(ims_remote_cb_t cb) { cb_pause_ = cb; }
  void onSave(ims_remote_cb_t cb) { cb_save_ = cb; }

private:
  ims_remote_cb_t cb_start_ = nullptr;
  ims_remote_cb_t cb_pause_ = nullptr;
  ims_remote_cb_t cb_save_  = nullptr;

  void setupRoutes_();
};
