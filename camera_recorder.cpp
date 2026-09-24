#include "camera_recorder.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <inttypes.h>
#include <new>
#include <string_view>
#include <sys/stat.h>
#include <vector>

#include "driver/sdmmc_host.h"
#include "dl_image_define.hpp"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_jpeg_dec.h"
#include "pedestrian_detect.hpp"

namespace esphome::camera_recorder {

static const char *const TAG = "camera_recorder";
static const char *const MOUNT_POINT = "/sdcard";
static constexpr uint32_t AVI_HEADER_SIZE = 224;
static constexpr uint32_t STORAGE_INTERVAL_MS = 5 * 60 * 1000;
static constexpr uint32_t WEB_RANGE_MAX_BYTES = 1024 * 1024;
static constexpr size_t WEB_FILE_BUFFER_SIZE = 16 * 1024;
static constexpr size_t RECORDINGS_PER_PAGE = 50;
static constexpr uint8_t MOTION_WARMUP_FRAMES = 3;
static constexpr uint8_t MOTION_HISTORY_FRAMES = 4;
static constexpr uint8_t MOTION_PIXEL_DIFFERENCE = 20;
static constexpr uint32_t MOTION_TASK_STACK_SIZE = 6144;
static constexpr size_t MOTION_JPEG_INITIAL_CAPACITY = 512 * 1024;
static constexpr size_t MOTION_COPY_CHUNK_SIZE = 8 * 1024;
static constexpr uint16_t CLASSIFICATION_MIN_SIZE = 224;
static const char *const MODEL_DIRECTORY = "/sdcard/models/s3";
static const char *const PERSON_MODEL = "pedestrian_detect_pico_s8_v1.espdl";

static const char RECORDINGS_CSS[] = R"css(
:root{color-scheme:light dark;--bg:#fff;--side:#f7f9fc;--card:#fff;--text:#172033;--muted:#697386;--line:#dce1e8;--accent:#0878ff;--active:#dcecff;--danger:#bd2c2c;--danger-active:#fff1f0;--shadow:0 8px 30px #18243a12;font:15px system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text)}a{color:inherit;text-decoration:none}.top{height:68px;padding:0 22px;border-bottom:1px solid var(--line);display:flex;align-items:center;justify-content:space-between;gap:16px}.identity,.statuses{display:flex;align-items:center;gap:8px;min-width:0}.host{font-size:20px;font-weight:750}.status,.storage,.pill,.download{border:1px solid var(--line);border-radius:999px;padding:8px 13px;background:var(--card)}.status,.storage{color:var(--muted);white-space:nowrap}.status.on{border-color:var(--accent);background:var(--active);color:var(--accent)}button.status{font:inherit;cursor:pointer}button.status:disabled{cursor:wait;opacity:.6}.layout{display:grid;grid-template-columns:230px minmax(0,1fr);min-height:calc(100vh - 68px)}.dates{background:var(--side);border-right:1px solid var(--line);padding:18px 12px;display:flex;flex-direction:column;gap:5px}.date{padding:10px 13px;border-radius:11px;color:var(--accent);font-weight:650}.date:hover,.date.active{background:var(--active)}main{min-width:0;padding:24px}.player{max-width:1400px;background:var(--card);border:1px solid var(--line);border-radius:18px;box-shadow:var(--shadow);overflow:hidden}.player video{display:block;width:100%;max-height:70vh;background:#080a0d}.empty{min-height:260px;display:grid;place-items:center;color:var(--muted)}.playerbar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:13px 15px}.filename{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-weight:650}.download{color:var(--accent);font-weight:650;white-space:nowrap}.clips{max-width:1400px;display:flex;flex-wrap:wrap;gap:9px;padding-top:18px}.pill{color:var(--accent);display:grid;gap:2px}.filesize{color:var(--muted);font-size:12px}.pill:hover,.pill.active{border-color:var(--accent);background:var(--active)}.pages{max-width:1400px;display:flex;align-items:center;justify-content:center;gap:12px;padding-top:18px;color:var(--muted)}.pages a{color:var(--accent)}
.playerbar{flex-wrap:wrap}.actions{display:flex;flex-wrap:wrap;gap:8px}.danger{border:1px solid var(--danger);border-radius:999px;padding:8px 13px;background:var(--card);color:var(--danger);font:inherit;font-weight:650;white-space:nowrap;cursor:pointer}.danger:hover{background:var(--danger-active)}.danger:focus-visible{outline:2px solid var(--danger);outline-offset:2px}.danger:disabled{opacity:.6;cursor:wait}.filename{min-width:0}
@media(prefers-color-scheme:dark){:root{--bg:#10141b;--side:#151a22;--card:#171d27;--text:#edf3ff;--muted:#9aa7bb;--line:#2c3544;--accent:#65a8ff;--active:#1b385c;--danger:#ff7979;--danger-active:#442126;--shadow:none}}
@media(max-width:700px){.top{height:58px;padding:0 14px}.host{display:none}.statuses{overflow-x:auto}.status,.storage{padding:7px 10px}.layout{display:block;min-height:calc(100vh - 58px)}.dates{border:0;border-bottom:1px solid var(--line);padding:10px;flex-direction:row;overflow-x:auto}.date{white-space:nowrap;padding:8px 11px}main{padding:12px}.player{border-radius:13px}.player video{max-height:54vh}.clips{padding-top:12px;gap:7px}.pill{padding:8px 11px}.playerbar{padding:11px 12px}}
)css";

static const char RECORDINGS_JS[] = R"js(
const b=document.getElementById('record-toggle'),deletes=document.querySelectorAll('[data-delete]');
let deleting=false,deleteDate='',deleteDay=false;
const refresh=()=>fetch('/recordings/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{
  const recording=document.getElementById('recording'),motion=document.getElementById('motion');
  recording.textContent=s.recording?'Recording':'Not recording';motion.textContent=s.motion?'Motion detected':'No motion';
  b.textContent=s.recording?'Stop & classify':'Start recording';recording.classList.toggle('on',s.recording);
  motion.classList.toggle('on',s.motion);b.classList.toggle('on',s.recording);
  if(deleting&&s.delete===3)location.href=deleteDay?'/recordings':'/recordings?date='+encodeURIComponent(deleteDate);
  if(deleting&&(s.delete===4||s.delete===0)){deleting=false;deletes.forEach(x=>x.disabled=false);alert('Deletion failed or was interrupted. Check the device logs.');location.reload()}
}).catch(()=>{});
b.onclick=()=>{b.disabled=true;fetch('/recordings/toggle',{method:'POST'}).finally(()=>{b.disabled=false;setTimeout(refresh,250)})};
deletes.forEach(button=>button.onclick=async()=>{
  const day=button.dataset.delete==='day',date=button.dataset.date,file=button.dataset.file;
  if(!confirm(day?'Delete every AVI from '+date+'?':'Delete '+file+'?'))return;
  const params=new URLSearchParams({date});if(!day)params.set('file',file);
  const label=button.textContent;button.textContent='Deleting…';
  deletes.forEach(x=>x.disabled=true);
  try{
    const response=await fetch('/recordings/delete?'+params,{method:'POST'});
    if(!response.ok)throw new Error((await response.text()).trim());
    deleting=true;deleteDate=date;deleteDay=day;refresh();
  }catch(error){button.textContent=label;deletes.forEach(x=>x.disabled=false);alert(error.message||'Deletion request failed')}
});
refresh();setInterval(refresh,3000);
)js";

static bool is_directory(const std::string &path) {
  struct stat info {};
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

static bool is_digits(const char *value, size_t length) {
  if (strlen(value) != length)
    return false;
  for (size_t index = 0; index < length; index++)
    if (!std::isdigit(static_cast<unsigned char>(value[index])))
      return false;
  return true;
}

static bool is_avi_name(const std::string &name) {
  return name.size() > 4 && name.find('/') == std::string::npos && name.find('\\') == std::string::npos &&
         name.find("..") == std::string::npos && name.compare(name.size() - 4, 4, ".avi") == 0;
}

static std::string html_escape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (char c : value) {
    switch (c) {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '"': escaped += "&quot;"; break;
      case '\'': escaped += "&#39;"; break;
      default: escaped += c;
    }
  }
  return escaped;
}

static std::string url_encode(const std::string &value) {
  static const char HEX[] = "0123456789ABCDEF";
  std::string encoded;
  encoded.reserve(value.size() * 3);
  for (const unsigned char c : value) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += HEX[c >> 4];
      encoded += HEX[c & 0x0F];
    }
  }
  return encoded;
}

static std::vector<std::string> list_recording_dates(const std::string &base) {
  std::vector<std::string> dates;
  DIR *years = opendir(base.c_str());
  if (years == nullptr)
    return dates;
  while (dirent *year = readdir(years)) {
    if (strcmp(year->d_name, "unsynced") == 0) {
      if (is_directory(base + "/unsynced"))
        dates.emplace_back("unsynced");
      continue;
    }
    if (!is_digits(year->d_name, 4) || !is_directory(base + "/" + year->d_name))
      continue;
    const std::string year_path = base + "/" + year->d_name;
    DIR *months = opendir(year_path.c_str());
    if (months == nullptr)
      continue;
    while (dirent *month = readdir(months)) {
      if (!is_digits(month->d_name, 2) || !is_directory(year_path + "/" + month->d_name))
        continue;
      const std::string month_path = year_path + "/" + month->d_name;
      DIR *days = opendir(month_path.c_str());
      if (days == nullptr)
        continue;
      while (dirent *day = readdir(days))
        if (is_digits(day->d_name, 2) && is_directory(month_path + "/" + day->d_name))
          dates.emplace_back(std::string(year->d_name) + "/" + month->d_name + "/" + day->d_name);
      closedir(days);
    }
    closedir(months);
  }
  closedir(years);
  std::sort(dates.rbegin(), dates.rend());
  std::stable_partition(dates.begin(), dates.end(), [](const std::string &date) { return date != "unsynced"; });
  return dates;
}

struct RecordingFile {
  std::string name;
  uint64_t size;
};

static std::vector<RecordingFile> list_recordings(const std::string &path) {
  std::vector<RecordingFile> recordings;
  DIR *directory = opendir(path.c_str());
  if (directory == nullptr)
    return recordings;
  while (dirent *entry = readdir(directory)) {
    const std::string name = entry->d_name;
    struct stat info {};
    if (is_avi_name(name) && stat((path + "/" + name).c_str(), &info) == 0 && S_ISREG(info.st_mode))
      recordings.push_back({name, static_cast<uint64_t>(info.st_size)});
  }
  closedir(directory);
  std::sort(recordings.begin(), recordings.end(),
            [](const RecordingFile &left, const RecordingFile &right) { return left.name > right.name; });
  return recordings;
}

static void remove_empty_recordings(const std::string &base) {
  for (const auto &date : list_recording_dates(base)) {
    const std::string path = base + "/" + date;
    DIR *directory = opendir(path.c_str());
    if (directory == nullptr)
      continue;
    while (dirent *entry = readdir(directory)) {
      const std::string recording = path + "/" + entry->d_name;
      struct stat info {};
      if (!is_avi_name(entry->d_name) || stat(recording.c_str(), &info) != 0 || !S_ISREG(info.st_mode) ||
          info.st_size > AVI_HEADER_SIZE)
        continue;
      if (remove(recording.c_str()) != 0) {
        ESP_LOGW(TAG, "Could not remove empty recording %s: %s", recording.c_str(), strerror(errno));
        continue;
      }
      const std::string index = recording + ".idx";
      if (remove(index.c_str()) != 0 && errno != ENOENT)
        ESP_LOGW(TAG, "Could not remove empty recording index %s: %s", index.c_str(), strerror(errno));
      ESP_LOGI(TAG, "Removed empty recording %s", recording.c_str());
    }
    closedir(directory);
  }
}

static bool valid_date(const std::string &date) {
  if (date == "unsynced")
    return true;
  return date.size() == 10 && date[4] == '/' && date[7] == '/' &&
         is_digits(date.substr(0, 4).c_str(), 4) && is_digits(date.substr(5, 2).c_str(), 2) &&
         is_digits(date.substr(8, 2).c_str(), 2);
}

static bool parse_range(const std::string &header, uint64_t size, uint64_t *start, uint64_t *end) {
  if (header.compare(0, 6, "bytes=") != 0 || header.find(',') != std::string::npos || size == 0)
    return false;
  const std::string value = header.substr(6);
  const size_t dash = value.find('-');
  if (dash == std::string::npos || value.find('-', dash + 1) != std::string::npos)
    return false;
  char *tail = nullptr;
  errno = 0;
  if (dash == 0) {
    const uint64_t suffix = strtoull(value.c_str() + 1, &tail, 10);
    if (errno != 0 || tail == value.c_str() + 1 || *tail != '\0' || suffix == 0)
      return false;
    *start = suffix >= size ? 0 : size - suffix;
    *end = size - 1;
    return true;
  }
  *start = strtoull(value.c_str(), &tail, 10);
  if (errno != 0 || tail != value.c_str() + dash || *start >= size)
    return false;
  if (dash + 1 == value.size()) {
    *end = size - 1;
    return true;
  }
  errno = 0;
  *end = strtoull(value.c_str() + dash + 1, &tail, 10);
  if (errno != 0 || tail == value.c_str() + dash + 1 || *tail != '\0' || *end < *start)
    return false;
  *end = std::min(*end, size - 1);
  return true;
}

static bool send_all(httpd_req_t *request, const char *data, size_t length) {
  while (length > 0) {
    const int sent = httpd_send(request, data, length);
    if (sent <= 0)
      return false;
    data += sent;
    length -= sent;
  }
  return true;
}

static void send_text(AsyncWebServerRequest *request, const char *status, const char *body) {
  httpd_req_t *raw = *request;
  httpd_resp_set_status(raw, status);
  httpd_resp_set_type(raw, "text/plain");
  httpd_resp_send(raw, body, HTTPD_RESP_USE_STRLEN);
}

struct WebAccessGuard {
  explicit WebAccessGuard(std::atomic<bool> *active) : active(active) {}
  ~WebAccessGuard() { this->active->store(false); }
  std::atomic<bool> *active;
};

static std::string_view request_path(AsyncWebServerRequest *request) {
  httpd_req_t *raw = *request;
  const std::string_view uri(raw->uri);
  return uri.substr(0, uri.find('?'));
}

bool CameraRecorder::canHandle(AsyncWebServerRequest *request) const {
  const auto url = request_path(request);
  if (request->method() == HTTP_POST)
    return url == "/recordings/toggle" || url == "/recordings/delete";
  if (request->method() != HTTP_GET)
    return false;
  return url == "/" || url == "/recordings" || url == "/recordings/" || url == "/recordings.css" ||
         url == "/recordings.js" || url == "/recordings/status" || url == "/recordings/file";
}

void CameraRecorder::handleRequest(AsyncWebServerRequest *request) {
  const auto url = request_path(request);
  if (url == "/") {
    request->redirect("/recordings");
  } else if (url == "/recordings.css") {
    httpd_resp_set_hdr(*request, "Cache-Control", "no-store");
    std::string css = RECORDINGS_CSS;
    css += ":root{--accent:" + this->light_accent_ + ";--active:" + this->light_active_ +
           "}@media(prefers-color-scheme:dark){:root{--accent:" + this->dark_accent_ + ";--active:" +
           this->dark_active_ + "}}";
    request->send(200, "text/css; charset=utf-8", css.c_str());
  } else if (url == "/recordings.js") {
    httpd_resp_set_hdr(*request, "Cache-Control", "no-store");
    request->send(200, "text/javascript; charset=utf-8", RECORDINGS_JS);
  } else if (url == "/recordings/status") {
    char json[64];
    const bool recording = this->recording_.load() || this->recording_pending_.load();
    const bool motion = this->motion_active_.load();
    snprintf(json, sizeof(json), "{\"recording\":%s,\"motion\":%s,\"delete\":%u}",
             recording ? "true" : "false", motion ? "true" : "false",
             static_cast<unsigned>(this->web_delete_state_.load()));
    httpd_resp_set_hdr(*request, "Cache-Control", "no-store");
    request->send(200, "application/json", json);
  } else if (url == "/recordings/toggle") {
    if (this->classification_pending_.load()) {
      request->send(409, "text/plain", "Classification is in progress.\n");
      return;
    }
    this->web_toggle_requested_.store(true);
    send_text(request, "202 Accepted", "Accepted.\n");
  } else if (url == "/recordings/delete") {
    this->handle_delete_(request);
  } else if (url == "/recordings/file") {
    this->handle_file_(request);
  } else {
    this->handle_index_(request);
  }
}

void CameraRecorder::handle_index_(AsyncWebServerRequest *request) {
  if (this->web_active_.exchange(true)) {
    httpd_resp_set_hdr(*request, "Retry-After", "2");
    send_text(request, "503 Service Unavailable", "SD card is busy; retry shortly.\n");
    return;
  }
  if (this->recording_.load() || this->recording_pending_.load() || this->classification_pending_.load()) {
    this->web_active_.store(false);
    httpd_resp_set_hdr(*request, "Retry-After", "2");
    send_text(request, "503 Service Unavailable", "SD card is busy; retry shortly.\n");
    return;
  }
  WebAccessGuard guard(&this->web_active_);

  const std::string base = std::string(MOUNT_POINT) + this->directory_;
  const auto dates = list_recording_dates(base);
  std::string selected_date;
  if (auto *parameter = request->getParam("date"); parameter != nullptr && valid_date(parameter->value()) &&
                                                     std::find(dates.begin(), dates.end(), parameter->value()) != dates.end())
    selected_date = parameter->value();
  else if (!dates.empty())
    selected_date = dates.front();

  const auto recordings = selected_date.empty() ? std::vector<RecordingFile>{}
                                                  : list_recordings(base + "/" + selected_date);
  if (this->recording_pending_.load()) {
    httpd_resp_set_hdr(*request, "Retry-After", "2");
    send_text(request, "503 Service Unavailable", "Recording is starting; retry shortly.\n");
    return;
  }
  size_t page = 0;
  if (auto *parameter = request->getParam("page"); parameter != nullptr) {
    const std::string value = parameter->value();
    char *tail = nullptr;
    errno = 0;
    const unsigned long parsed = strtoul(value.c_str(), &tail, 10);
    if (errno == 0 && tail != value.c_str() && *tail == '\0')
      page = parsed;
  }
  const size_t page_count = std::max<size_t>(1, (recordings.size() + RECORDINGS_PER_PAGE - 1) / RECORDINGS_PER_PAGE);
  page = std::min(page, page_count - 1);
  const size_t page_begin = std::min(page * RECORDINGS_PER_PAGE, recordings.size());
  const size_t page_end = std::min(page_begin + RECORDINGS_PER_PAGE, recordings.size());
  std::string selected_file;
  if (auto *parameter = request->getParam("file"); parameter != nullptr && is_avi_name(parameter->value()) &&
                                                     std::find_if(recordings.begin(), recordings.end(), [&](const RecordingFile &recording) {
                                                       return recording.name == parameter->value();
                                                     }) != recordings.end())
    selected_file = parameter->value();
  else if (page_begin < page_end)
    selected_file = recordings[page_begin].name;

  char storage[48];
  const uint32_t free_mib = this->sd_free_mib_.load();
  if (this->sd_total_mib_.load() == 0)
    snprintf(storage, sizeof(storage), "SD unavailable");
  else
    snprintf(storage, sizeof(storage), "%.1f GiB free", free_mib / 1024.0f);

  std::string html =
      "<!doctype html><html><head><meta charset=utf-8><meta name=viewport "
      "content=\"width=device-width,initial-scale=1\"><title>";
  html += html_escape(App.get_name().c_str());
  html += " recordings</title><link rel=stylesheet href=/recordings.css?v=2><script defer src=/recordings.js?v=2></script></head><body><header class=top><div "
          "class=identity><div class=host>";
  html += html_escape(App.get_name().c_str());
  html += "</div><button id=record-toggle class=status type=button>Start recording</button></div><div class=statuses><span id=recording class=status>Not recording</span><span id=motion "
          "class=status>No motion</span><span class=storage>";
  html += storage;
  html += "</span></div></header><div class=layout><nav class=dates aria-label=\"Recording dates\">";
  for (const auto &date : dates) {
    html += "<a class=\"date";
    html += date == selected_date ? " active" : "";
    html += "\" href=\"/recordings?date=";
    html += url_encode(date);
    html += date == selected_date ? "\" aria-current=page>" : "\">";
    html += html_escape(date);
    html += "</a>";
  }
  html += "</nav><main><section class=player>";
  std::string file_url;
  if (selected_file.empty()) {
    html += "<div class=empty>No recordings for this date</div>";
    if (!selected_date.empty()) {
      html += "<div class=playerbar><div class=actions><button class=danger type=button data-delete=day data-date=\"";
      html += html_escape(selected_date);
      html += "\">Delete This Day</button></div></div>";
    }
  } else {
    file_url = "/recordings/file?path=" + url_encode(selected_date + "/" + selected_file);
    html += "<video controls playsinline preload=metadata src=\"";
    html += file_url;
    html += "\"></video><div class=playerbar><div class=actions><button class=danger type=button data-delete=file data-date=\"";
    html += html_escape(selected_date);
    html += "\" data-file=\"";
    html += html_escape(selected_file);
    html += "\">Delete AVI</button><button class=danger type=button data-delete=day data-date=\"";
    html += html_escape(selected_date);
    html += "\">Delete This Day</button><a class=download href=\"";
    html += file_url;
    html += "&amp;download=1\">Download AVI</a></div><div class=filename>";
    html += html_escape(selected_file);
    html += "</div></div>";
  }
  html += "</section><section class=clips aria-label=\"Recordings\">";
  for (size_t index = page_begin; index < page_end; index++) {
    const auto &recording = recordings[index];
    html += "<a class=\"pill";
    html += recording.name == selected_file ? " active" : "";
    html += "\" href=\"/recordings?date=";
    html += url_encode(selected_date);
    html += "&amp;file=";
    html += url_encode(recording.name);
    html += "&amp;page=";
    html += std::to_string(page);
    html += recording.name == selected_file ? "\" aria-current=true>" : "\">";
    html += "<span>";
    html += html_escape(recording.name);
    html += "</span>";
    char file_size[24];
    snprintf(file_size, sizeof(file_size), "%.2f MiB", recording.size / (1024.0f * 1024.0f));
    html += "<span class=filesize>";
    html += file_size;
    html += "</span></a>";
  }
  html += "</section>";
  if (page_count > 1) {
    html += "<nav class=pages aria-label=\"Recording pages\">";
    if (page > 0) {
      html += "<a href=\"/recordings?date=";
      html += url_encode(selected_date);
      html += "&amp;page=" + std::to_string(page - 1) + "\">Previous</a>";
    }
    html += "<span>Page " + std::to_string(page + 1) + " of " + std::to_string(page_count) + "</span>";
    if (page + 1 < page_count) {
      html += "<a href=\"/recordings?date=";
      html += url_encode(selected_date);
      html += "&amp;page=" + std::to_string(page + 1) + "\">Next</a>";
    }
    html += "</nav>";
  }
  html += "</main></div></body></html>";

  httpd_req_t *raw = *request;
  httpd_resp_set_type(raw, "text/html; charset=utf-8");
  httpd_resp_set_hdr(raw, "Cache-Control", "no-store");
  httpd_resp_set_hdr(raw, "Content-Security-Policy",
                     "default-src 'self'; style-src 'self'; media-src 'self'; img-src 'none'; script-src 'self'");
  if (httpd_resp_send(raw, html.data(), html.size()) != ESP_OK)
    ESP_LOGW(TAG, "Recordings page connection closed while sending");
}

void CameraRecorder::handle_delete_(AsyncWebServerRequest *request) {
  const auto origin = request->get_header("Origin");
  const auto host = request->get_header("Host");
  if (origin.has_value() && (!host.has_value() ||
                             (*origin != "http://" + *host && *origin != "https://" + *host))) {
    send_text(request, "403 Forbidden", "Cross-origin deletion is not allowed.\n");
    return;
  }
  auto *date = request->getParam("date");
  auto *file = request->getParam("file");
  if (date == nullptr || !valid_date(date->value()) || (file != nullptr && !is_avi_name(file->value()))) {
    request->send(400, "text/plain", "Invalid recording selection.\n");
    return;
  }
  if (this->web_active_.exchange(true)) {
    send_text(request, "503 Service Unavailable", "SD card is busy; retry shortly.\n");
    return;
  }
  WebAccessGuard guard(&this->web_active_);
  if (!this->mounted_) {
    send_text(request, "503 Service Unavailable", "SD card unavailable.\n");
    return;
  }
  const auto state = this->web_delete_state_.load();
  if (state == WebDeletionState::QUEUED || state == WebDeletionState::RUNNING) {
    request->send(409, "text/plain", "A deletion is already in progress.\n");
    return;
  }
  this->web_delete_date_ = date->value();
  this->web_delete_file_ = file == nullptr ? "" : file->value();
  this->web_delete_state_.store(WebDeletionState::QUEUED);
  send_text(request, "202 Accepted", "Deletion queued.\n");
}

void CameraRecorder::handle_file_(AsyncWebServerRequest *request) {
  if (this->web_active_.exchange(true)) {
    httpd_resp_set_hdr(*request, "Retry-After", "2");
    send_text(request, "503 Service Unavailable", "SD card is busy; retry shortly.\n");
    return;
  }
  if (this->recording_.load() || this->recording_pending_.load() || this->classification_pending_.load()) {
    this->web_active_.store(false);
    httpd_resp_set_hdr(*request, "Retry-After", "2");
    send_text(request, "503 Service Unavailable", "SD card is busy; retry shortly.\n");
    return;
  }
  WebAccessGuard guard(&this->web_active_);

  auto *parameter = request->getParam("path");
  if (parameter == nullptr) {
    request->send(400, "text/plain", "Missing recording path.\n");
    return;
  }
  const std::string relative = parameter->value();
  const size_t slash = relative.rfind('/');
  const std::string date = slash == std::string::npos ? "" : relative.substr(0, slash);
  const std::string name = slash == std::string::npos ? "" : relative.substr(slash + 1);
  if (!valid_date(date) || !is_avi_name(name)) {
    request->send(400, "text/plain", "Invalid recording path.\n");
    return;
  }

  const std::string path = std::string(MOUNT_POINT) + this->directory_ + "/" + date + "/" + name;
  struct stat info {};
  if (stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) {
    request->send(404, "text/plain", "Recording not found.\n");
    return;
  }
  const uint64_t size = info.st_size;
  uint64_t start = 0;
  uint64_t end = size == 0 ? 0 : size - 1;
  bool partial = false;
  if (const auto range = request->get_header("Range"); range.has_value()) {
    partial = true;
    if (!parse_range(*range, size, &start, &end)) {
      char content_range[48];
      snprintf(content_range, sizeof(content_range), "bytes */%" PRIu64, size);
      httpd_resp_set_status(*request, "416 Range Not Satisfiable");
      httpd_resp_set_hdr(*request, "Content-Range", content_range);
      httpd_resp_send(*request, nullptr, 0);
      return;
    }
    end = std::min<uint64_t>(end, start + WEB_RANGE_MAX_BYTES - 1);
  }

  FILE *file = fopen(path.c_str(), "rb");
  auto *buffer = static_cast<char *>(heap_caps_malloc(WEB_FILE_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (file == nullptr || buffer == nullptr || fseeko(file, static_cast<off_t>(start), SEEK_SET) != 0) {
    if (file != nullptr)
      fclose(file);
    heap_caps_free(buffer);
    send_text(request, "503 Service Unavailable", "Could not open recording.\n");
    return;
  }

  const uint64_t length = size == 0 ? 0 : end - start + 1;
  size_t used = snprintf(buffer, WEB_FILE_BUFFER_SIZE,
                         "HTTP/1.1 %s\r\nContent-Type: video/x-msvideo\r\nContent-Length: %" PRIu64
                         "\r\nAccept-Ranges: bytes\r\nCache-Control: no-store\r\nConnection: close\r\n",
                         partial ? "206 Partial Content" : "200 OK", length);
  if (partial)
    used += snprintf(buffer + used, WEB_FILE_BUFFER_SIZE - used, "Content-Range: bytes %" PRIu64 "-%" PRIu64 "/%" PRIu64 "\r\n",
                     start, end, size);
  if (request->hasParam("download"))
    used += snprintf(buffer + used, WEB_FILE_BUFFER_SIZE - used,
                     "Content-Disposition: attachment; filename=\"recording.avi\"\r\n");
  used += snprintf(buffer + used, WEB_FILE_BUFFER_SIZE - used, "\r\n");

  httpd_req_t *raw = *request;
  bool ok = used < WEB_FILE_BUFFER_SIZE && send_all(raw, buffer, used);
  uint64_t remaining = length;
  bool preempted = false;
  while (ok && remaining > 0) {
    if (this->recording_pending_.load()) {
      preempted = true;
      break;
    }
    const size_t chunk = std::min<uint64_t>(WEB_FILE_BUFFER_SIZE, remaining);
    const size_t read = fread(buffer, 1, chunk, file);
    if (read != chunk || !send_all(raw, buffer, read)) {
      ok = false;
      break;
    }
    remaining -= read;
    vTaskDelay(1);
  }
  fclose(file);
  heap_caps_free(buffer);
  if (!ok && !preempted)
    ESP_LOGW(TAG, "Recording stream ended early: %s", path.c_str());
}

struct ClassificationState {
  FILE *file{nullptr};
  uint32_t index_offset{0};
  uint32_t last_sample_end_ms{0};
  uint32_t samples[4]{};
  uint8_t sample_count{0};
  uint8_t next_sample{0};
  uint8_t detections{0};
};

void CameraRecorderNumber::setup_restore(float default_value) {
  this->preference_ = this->make_entity_preference<float>(1);
  float value;
  if (!this->preference_.load(&value) || !std::isfinite(value))
    value = default_value;
  this->apply_(std::clamp(value, this->traits.get_min_value(), this->traits.get_max_value()));
}

void CameraRecorderNumber::control(float value) {
  value = std::clamp(value, this->traits.get_min_value(), this->traits.get_max_value());
  this->apply_(value);
  if (!this->preference_.save(&value))
    ESP_LOGW(TAG, "Failed to save %s", this->get_name().c_str());
}

void CameraRecorderNumber::apply_(float value) {
  if (this->type_ == CameraRecorderNumberType::THRESHOLD)
    this->parent_->set_motion_threshold(value);
  else if (this->type_ == CameraRecorderNumberType::HOLD_TIME)
    this->parent_->set_motion_hold_time(value);
  else if (this->type_ == CameraRecorderNumberType::MOTION_FPS)
    this->parent_->set_motion_fps(value);
  else {
    value = std::round(value);
    this->parent_->set_min_person_detections(static_cast<uint8_t>(value));
  }
  this->publish_state(value);
}

void CameraRecorderSwitch::setup_restore(bool default_value) {
  this->write_state(this->get_initial_state_with_restore_mode().value_or(default_value));
}

void CameraRecorderSwitch::write_state(bool state) {
  if (this->type_ == CameraRecorderSwitchType::PEOPLE)
    this->parent_->set_record_people(state);
  else if (this->type_ == CameraRecorderSwitchType::OTHER_MOTION)
    this->parent_->set_record_other_motion(state);
  else if (this->type_ == CameraRecorderSwitchType::AUTO_DELETION)
    this->parent_->set_auto_deletion(state);
  else
    this->parent_->set_motion_detection(state);
  this->publish_state(state);
}

void CameraRecorder::add_record_switch(CameraRecorderSwitch *record_switch) {
  if (record_switch->get_type() == CameraRecorderSwitchType::PEOPLE)
    this->record_people_switch_ = record_switch;
  else if (record_switch->get_type() == CameraRecorderSwitchType::OTHER_MOTION)
    this->record_other_motion_switch_ = record_switch;
  else if (record_switch->get_type() == CameraRecorderSwitchType::AUTO_DELETION)
    this->auto_deletion_switch_ = record_switch;
  else
    this->motion_detection_switch_ = record_switch;
}

void CameraRecorder::set_motion_detection(bool state) {
  this->motion_detection_enabled_ = state;
  this->motion_reset_pending_ = true;
  if (!state) {
    this->publish_motion_(false);
    this->motion_score_sensor_->publish_state(0.0f);
  }
}

static void put_u16(uint8_t *data, size_t offset, uint16_t value) {
  data[offset] = value;
  data[offset + 1] = value >> 8;
}

static void put_u32(uint8_t *data, size_t offset, uint32_t value) {
  data[offset] = value;
  data[offset + 1] = value >> 8;
  data[offset + 2] = value >> 16;
  data[offset + 3] = value >> 24;
}

static uint32_t get_u32(const uint8_t *data, size_t offset) {
  return static_cast<uint32_t>(data[offset]) | (static_cast<uint32_t>(data[offset + 1]) << 8) |
         (static_cast<uint32_t>(data[offset + 2]) << 16) | (static_cast<uint32_t>(data[offset + 3]) << 24);
}

static bool jpeg_dimensions(const uint8_t *data, size_t length, uint16_t *width, uint16_t *height) {
  for (size_t offset = 2; offset + 4 <= length;) {
    if (data[offset++] != 0xFF)
      continue;
    while (offset < length && data[offset] == 0xFF)
      offset++;
    if (offset >= length)
      break;
    const uint8_t marker = data[offset++];
    if (marker == 0xD8 || marker == 0xD9)
      continue;
    if (marker == 0xDA || offset + 2 > length)
      break;
    const uint16_t segment_length = (data[offset] << 8) | data[offset + 1];
    if (segment_length < 2 || offset + segment_length > length)
      break;
    const bool is_sof = marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
    if (is_sof && segment_length >= 7) {
      *height = (data[offset + 3] << 8) | data[offset + 4];
      *width = (data[offset + 5] << 8) | data[offset + 6];
      return *width != 0 && *height != 0;
    }
    offset += segment_length;
  }
  return false;
}

static uint8_t classification_divisor(uint16_t width, uint16_t height) {
  if (width / 8 >= CLASSIFICATION_MIN_SIZE && height / 8 >= CLASSIFICATION_MIN_SIZE)
    return 8;
  if (width / 4 >= CLASSIFICATION_MIN_SIZE && height / 4 >= CLASSIFICATION_MIN_SIZE)
    return 4;
  if (width / 2 >= CLASSIFICATION_MIN_SIZE && height / 2 >= CLASSIFICATION_MIN_SIZE)
    return 2;
  return 1;
}

static dl::image::img_t decode_classification_frame(const uint8_t *jpeg, size_t length) {
  dl::image::img_t image{};
  uint16_t width;
  uint16_t height;
  if (length > INT_MAX || !jpeg_dimensions(jpeg, length, &width, &height))
    return image;

  const uint8_t divisor = classification_divisor(width, height);
  const uint16_t output_width = divisor == 1 ? width : (width / divisor) & ~7U;
  const uint16_t output_height = divisor == 1 ? height : (height / divisor) & ~7U;
  jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
  config.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
  if (divisor != 1) {
    config.scale.width = output_width;
    config.scale.height = output_height;
  }

  jpeg_dec_handle_t decoder = nullptr;
  if (jpeg_dec_open(&config, &decoder) == JPEG_ERR_OK) {
    jpeg_dec_io_t io{};
    io.inbuf = const_cast<uint8_t *>(jpeg);
    io.inbuf_len = static_cast<int>(length);
    jpeg_dec_header_info_t header{};
    int output_len = 0;
    if (jpeg_dec_parse_header(decoder, &io, &header) == JPEG_ERR_OK &&
        jpeg_dec_get_outbuf_len(decoder, &output_len) == JPEG_ERR_OK && output_len > 0 &&
        static_cast<size_t>(output_len) == static_cast<size_t>(output_width) * output_height * 2) {
      image.data = heap_caps_aligned_alloc(16, output_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (image.data != nullptr) {
        io.outbuf = static_cast<uint8_t *>(image.data);
        if (jpeg_dec_process(decoder, &io) == JPEG_ERR_OK && io.out_size == output_len) {
          image.width = output_width;
          image.height = output_height;
          image.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
        } else {
          heap_caps_free(image.data);
          image.data = nullptr;
        }
      }
    }
    jpeg_dec_close(decoder);
  }

  if (image.data != nullptr) {
    ESP_LOGD(TAG, "Classification frame (new JPEG): %ux%u RGB565 (%u bytes)", image.width, image.height,
             static_cast<unsigned>(static_cast<size_t>(image.width) * image.height * 2));
    return image;
  }
  ESP_LOGW(TAG, "Failed to decode classification JPEG");
  return image;
}

static bool patch_u32(FILE *file, long offset, uint32_t value) {
  uint8_t data[4];
  put_u32(data, 0, value);
  return fseek(file, offset, SEEK_SET) == 0 && fwrite(data, 1, sizeof(data), file) == sizeof(data);
}

void CameraRecorder::setup() {
  if (this->mounted_sensor_ != nullptr)
    this->mounted_sensor_->publish_initial_state(false);
  this->publish_recording_(false);
  this->classifying_sensor_->publish_initial_state(false);
  this->models_ready_sensor_->publish_initial_state(false);
  this->person_detected_sensor_->publish_initial_state(false);
  this->motion_sensor_->publish_initial_state(false);
  this->motion_score_sensor_->publish_state(0.0f);
  this->threshold_number_->setup_restore(18.0f);
  this->hold_time_number_->setup_restore(10.0f);
  this->motion_fps_number_->setup_restore(0.8f);
  this->min_person_detections_number_->setup_restore(2.0f);
  this->record_people_switch_->setup_restore(true);
  this->record_other_motion_switch_->setup_restore(true);
  this->auto_deletion_switch_->setup_restore(true);
  this->motion_detection_switch_->setup_restore(true);
  this->camera_->add_listener(this);
  this->web_server_->add_handler(this);
  // lwIP is initialized later than this hardware-priority component.
  this->defer([this]() { this->web_server_->init(); });

  if (!this->mount_()) {
    this->mark_failed();
    return;
  }

  this->mounted_ = true;
  if (this->mounted_sensor_ != nullptr)
    this->mounted_sensor_->publish_state(true);
  remove_empty_recordings(std::string(MOUNT_POINT) + this->directory_);
  if (this->models_ready_()) {
    ESP_LOGI(TAG, "Loading pedestrian model; internal heap=%u, largest block=%u",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    this->detector_ = new (std::nothrow) PedestrianDetect(PedestrianDetect::PICO_S8_V1, false);
    ESP_LOGI(TAG, "Pedestrian model loaded; internal heap=%u, largest block=%u",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  }
  this->models_ready_sensor_->publish_state(this->detector_ != nullptr);
  this->motion_jpeg_ = static_cast<uint8_t *>(
      heap_caps_malloc(MOTION_JPEG_INITIAL_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->motion_jpeg_ != nullptr)
    this->motion_jpeg_size_ = MOTION_JPEG_INITIAL_CAPACITY;
  else
    ESP_LOGW(TAG, "Could not preallocate motion JPEG buffer");
  this->motion_task_running_ = true;
  if (xTaskCreatePinnedToCore(&CameraRecorder::motion_task_, "motion_detect", MOTION_TASK_STACK_SIZE, this, 0,
                              &this->motion_task_handle_, 1) != pdPASS) {
    this->motion_task_running_ = false;
    this->motion_task_handle_ = nullptr;
    ESP_LOGE(TAG, "Could not start motion detection task");
  }
  this->update_storage_();
}

void CameraRecorder::loop() {
  this->publish_motion_result_();
  if (this->web_toggle_requested_.exchange(false)) {
    if (!this->classification_pending_) {
      if (this->recording_ || this->recording_pending_)
        this->stop_and_classify();
      else
        this->start();
    }
    return;
  }
  if (this->classification_pending_) {
    if (this->motion_busy_)
      return;
    this->run_classification_();
    return;
  }

  if (this->recording_pending_ && !this->web_active_) {
    this->start();
    return;
  }

  if (!this->recording_ && !this->recording_pending_)
    this->prune_step_();

  const uint32_t now = millis();
  if (this->recording_) {
    this->camera_->request_image(camera::WEB_REQUESTER);
  } else if (this->motion_detection_enabled_ && !this->motion_busy_ &&
             now - this->last_motion_request_ms_ >= this->motion_interval_ms_) {
    this->last_motion_request_ms_ = now;
    this->camera_->request_image(camera::WEB_REQUESTER);
  }

  if (this->motion_active_ && now - this->last_motion_detected_ms_ >= this->motion_hold_time_s_ * 1000)
    this->publish_motion_(false);

  if (!this->recording_.load() && !this->classification_pending_.load() && !this->web_active_.load() &&
      now - this->last_storage_update_ms_ >= STORAGE_INTERVAL_MS)
    this->update_storage_();
}

void CameraRecorder::update_storage_() {
  if (!this->mounted_)
    return;
  uint64_t total = 0;
  uint64_t free = 0;
  const esp_err_t error = esp_vfs_fat_info(MOUNT_POINT, &total, &free);
  this->last_storage_update_ms_ = millis();
  if (error != ESP_OK) {
    ESP_LOGW(TAG, "Could not read SD storage: %s", esp_err_to_name(error));
    return;
  }
  this->sd_total_mib_.store(total / (1024 * 1024));
  this->sd_free_mib_.store(free / (1024 * 1024));
  this->sd_free_sensor_->publish_state(free / (1024.0f * 1024.0f * 1024.0f));
}

void CameraRecorder::delete_oldest_days(uint8_t days) {
  this->manual_delete_days_ = std::min<uint16_t>(255, this->manual_delete_days_ + days);
}

void CameraRecorder::prune_step_() {
  if (!this->mounted_ ||
      (this->manual_delete_days_ == 0 && !this->auto_delete_pending_ && this->prune_date_.empty() &&
       this->web_delete_state_.load() != WebDeletionState::QUEUED) ||
      this->web_active_.exchange(true))
    return;
  WebAccessGuard guard(&this->web_active_);
  const std::string base = std::string(MOUNT_POINT) + this->directory_;
  if (this->prune_date_.empty() && this->web_delete_state_.load() == WebDeletionState::QUEUED) {
    const std::string path = base + "/" + this->web_delete_date_;
    if (!this->web_delete_file_.empty()) {
      const std::string avi = path + "/" + this->web_delete_file_;
      struct stat info {};
      if (stat(avi.c_str(), &info) != 0 || !S_ISREG(info.st_mode) || remove(avi.c_str()) != 0) {
        ESP_LOGE(TAG, "Could not delete recording %s: %s", avi.c_str(), strerror(errno));
        this->web_delete_state_.store(WebDeletionState::FAILED);
      } else {
        const std::string index = avi + ".idx";
        if (stat(index.c_str(), &info) == 0 && S_ISREG(info.st_mode) && remove(index.c_str()) != 0)
          ESP_LOGW(TAG, "Could not delete recording index %s: %s", index.c_str(), strerror(errno));
        this->web_delete_state_.store(WebDeletionState::DONE);
        this->update_storage_();
      }
      return;
    }
    if (!is_directory(path)) {
      this->web_delete_state_.store(WebDeletionState::FAILED);
      return;
    }
    this->prune_date_ = this->web_delete_date_;
    this->prune_removed_any_ = false;
    this->web_delete_state_.store(WebDeletionState::RUNNING);
    ESP_LOGI(TAG, "Deleting selected recording day %s", this->prune_date_.c_str());
  }
  if (this->prune_date_.empty()) {
    const auto now = this->time_->now();
    if (!now.is_valid())
      return;
    if (this->auto_delete_pending_ && this->manual_delete_days_ == 0) {
      uint64_t total = 0;
      uint64_t free = 0;
      if (esp_vfs_fat_info(MOUNT_POINT, &total, &free) != ESP_OK) {
        ESP_LOGW(TAG, "Could not check SD storage for auto deletion");
        this->auto_delete_pending_ = false;
        return;
      }
      if (total == 0 || free >= (total + 9) / 10) {
        this->auto_delete_pending_ = false;
        return;
      }
    }
    char today[16];
    snprintf(today, sizeof(today), "%04u/%02u/%02u", now.year, now.month, now.day_of_month);
    const auto dates = list_recording_dates(base);
    for (auto date = dates.rbegin(); date != dates.rend(); ++date) {
      if (date->size() == 10 && *date < today) {
        this->prune_date_ = *date;
        this->prune_removed_any_ = false;
        break;
      }
    }
    if (this->prune_date_.empty()) {
      this->manual_delete_days_ = 0;
      this->auto_delete_pending_ = false;
      return;
    }
    ESP_LOGI(TAG, "Deleting oldest recording day %s", this->prune_date_.c_str());
  }
  const std::string path = base + "/" + this->prune_date_;
  if (this->prune_dir_ == nullptr)
    this->prune_dir_ = opendir(path.c_str());
  if (this->prune_dir_ == nullptr) {
    ESP_LOGE(TAG, "Could not open recording day %s: %s", path.c_str(), strerror(errno));
    this->prune_date_.clear();
    if (this->web_delete_state_.load() == WebDeletionState::RUNNING)
      this->web_delete_state_.store(WebDeletionState::FAILED);
    else {
      this->manual_delete_days_ = 0;
      this->auto_delete_pending_ = false;
    }
    return;
  }
  while (dirent *entry = readdir(this->prune_dir_)) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..")
      continue;
    const bool index = name.size() > 8 && name.compare(name.size() - 8, 8, ".avi.idx") == 0 &&
                       is_avi_name(name.substr(0, name.size() - 4));
    if (!is_avi_name(name) && !index)
      continue;
    const std::string file = path + "/" + name;
    struct stat info {};
    if (stat(file.c_str(), &info) != 0 || !S_ISREG(info.st_mode) || remove(file.c_str()) != 0) {
      ESP_LOGE(TAG, "Could not delete %s: %s", file.c_str(), strerror(errno));
      closedir(this->prune_dir_);
      this->prune_dir_ = nullptr;
      this->prune_date_.clear();
      if (this->web_delete_state_.load() == WebDeletionState::RUNNING)
        this->web_delete_state_.store(WebDeletionState::FAILED);
      else {
        this->manual_delete_days_ = 0;
        this->auto_delete_pending_ = false;
      }
    } else
      this->prune_removed_any_ = true;
    return;
  }
  closedir(this->prune_dir_);
  this->prune_dir_ = nullptr;
  if (rmdir(path.c_str()) != 0) {
    if (this->prune_removed_any_) {
      this->prune_removed_any_ = false;
      return;
    }
    ESP_LOGE(TAG, "Could not remove recording day %s: %s", path.c_str(), strerror(errno));
    if (this->web_delete_state_.load() == WebDeletionState::RUNNING)
      this->web_delete_state_.store(WebDeletionState::FAILED);
    else {
      this->manual_delete_days_ = 0;
      this->auto_delete_pending_ = false;
    }
  } else {
    ESP_LOGI(TAG, "Deleted recording day %s", this->prune_date_.c_str());
    if (this->prune_date_.size() == 10) {
      const std::string month = path.substr(0, path.size() - 3);
      rmdir(month.c_str());
      const std::string year = month.substr(0, month.size() - 3);
      rmdir(year.c_str());
    }
    if (this->web_delete_state_.load() == WebDeletionState::RUNNING)
      this->web_delete_state_.store(WebDeletionState::DONE);
    else if (this->manual_delete_days_ != 0)
      this->manual_delete_days_--;
  }
  this->prune_date_.clear();
  this->update_storage_();
}

bool CameraRecorder::mount_() {
  esp_vfs_fat_sdmmc_mount_config_t mount_config{};
  mount_config.format_if_mount_failed = this->format_if_mount_failed_;
  mount_config.max_files = 4;
  mount_config.allocation_unit_size = 16 * 1024;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk = static_cast<gpio_num_t>(this->clk_pin_);
  slot.cmd = static_cast<gpio_num_t>(this->cmd_pin_);
  slot.d0 = static_cast<gpio_num_t>(this->data0_pin_);
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  const esp_err_t error = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount_config, &this->card_);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(error));
    return false;
  }

  const std::string path = std::string(MOUNT_POINT) + this->directory_;
  if (this->directory_ != "/" && !this->ensure_directory_(path)) {
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, this->card_);
    this->card_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "Mounted SD card at %s", MOUNT_POINT);
  return true;
}

void CameraRecorder::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Camera Recorder:\n"
                "  SDMMC pins: CLK=%u CMD=%u DATA0=%u\n"
                "  Directory: %s\n"
                "  Format if mount fails: %s\n"
                "  Mounted: %s",
                this->clk_pin_, this->cmd_pin_, this->data0_pin_, this->directory_.c_str(),
                YESNO(this->format_if_mount_failed_), YESNO(this->mounted_));
  LOG_BINARY_SENSOR("  ", "Mounted", this->mounted_sensor_);
  LOG_BINARY_SENSOR("  ", "Recording", this->recording_sensor_);
  LOG_BINARY_SENSOR("  ", "Classifying", this->classifying_sensor_);
  LOG_BINARY_SENSOR("  ", "Person Model Ready", this->models_ready_sensor_);
  LOG_BINARY_SENSOR("  ", "Person Detected", this->person_detected_sensor_);
  LOG_BINARY_SENSOR("  ", "Motion", this->motion_sensor_);
  LOG_SENSOR("  ", "Motion Score", this->motion_score_sensor_);
  LOG_SENSOR("  ", "SD Card Free", this->sd_free_sensor_);
  LOG_NUMBER("  ", "Motion Threshold", this->threshold_number_);
  LOG_NUMBER("  ", "Motion Hold Time", this->hold_time_number_);
  LOG_NUMBER("  ", "Motion Detection FPS", this->motion_fps_number_);
  LOG_NUMBER("  ", "Minimum Person Detections", this->min_person_detections_number_);
  LOG_SWITCH("  ", "Record People", this->record_people_switch_);
  LOG_SWITCH("  ", "Record Other Motion", this->record_other_motion_switch_);
  LOG_SWITCH("  ", "Auto Deletion", this->auto_deletion_switch_);
}

bool CameraRecorder::ensure_directory_(const std::string &path) {
  for (size_t offset = 1; offset <= path.size(); offset++) {
    if (offset != path.size() && path[offset] != '/')
      continue;
    const std::string part = path.substr(0, offset);
    if (!part.empty() && mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) {
      ESP_LOGE(TAG, "Failed to create %s: %s", part.c_str(), strerror(errno));
      return false;
    }
  }
  return true;
}

std::string CameraRecorder::unique_filename_(const std::string &stem, const char *suffix) const {
  struct stat info {};
  std::string candidate = stem + suffix;
  if (stat(candidate.c_str(), &info) != 0 && errno == ENOENT)
    return candidate;

  for (unsigned index = 1; index <= 999; index++) {
    char numbered[8];
    snprintf(numbered, sizeof(numbered), "-%02u", index);
    candidate = stem + numbered + suffix;
    if (stat(candidate.c_str(), &info) != 0 && errno == ENOENT)
      return candidate;
  }
  return {};
}

bool CameraRecorder::open_next_file_() {
  std::string base = std::string(MOUNT_POINT) + this->directory_;
  std::string stem;
  const auto now = this->time_->now();
  if (now.is_valid()) {
    char dated[40];
    snprintf(dated, sizeof(dated), "/%04u/%02u/%02u", now.year, now.month, now.day_of_month);
    base += dated;
    if (!this->ensure_directory_(base))
      return false;
    char timestamp[16];
    snprintf(timestamp, sizeof(timestamp), "%02u-%02u-%02u", now.hour, now.minute, now.second);
    stem = base + "/" + timestamp;
  } else {
    base += "/unsynced";
    if (!this->ensure_directory_(base))
      return false;
    stem = base + "/recording";
  }

  this->filename_ = this->unique_filename_(stem, ".avi");
  if (this->filename_.empty()) {
    ESP_LOGE(TAG, "No free recording filename remains");
    return false;
  }
  this->filename_stem_ = this->filename_.substr(0, this->filename_.size() - 4);
  this->file_ = fopen(this->filename_.c_str(), "wb");
  if (this->file_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create %s: %s", this->filename_.c_str(), strerror(errno));
    return false;
  }
  this->index_filename_ = this->filename_ + ".idx";
  this->index_file_ = fopen(this->index_filename_.c_str(), "w+b");
  if (this->index_file_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create AVI index: %s", strerror(errno));
    fclose(this->file_);
    this->file_ = nullptr;
    remove(this->filename_.c_str());
    return false;
  }
  return true;
}

void CameraRecorder::start() {
  if (this->prune_dir_ != nullptr) {
    closedir(this->prune_dir_);
    this->prune_dir_ = nullptr;
  }
  if (this->web_delete_state_.load() == WebDeletionState::RUNNING) {
    ESP_LOGW(TAG, "Stopped day deletion to start recording");
    this->prune_date_.clear();
    this->web_delete_state_.store(WebDeletionState::FAILED);
  }
  if (this->recording_) {
    this->recording_pending_ = false;
    ESP_LOGW(TAG, "Already recording");
    return;
  }
  this->recording_pending_ = true;
  if (this->web_active_.exchange(true)) {
    ESP_LOGI(TAG, "Recording queued while SD card web access stops");
    return;
  }
  WebAccessGuard guard(&this->web_active_);
  if (this->classification_pending_) {
    ESP_LOGI(TAG, "Recording queued until classification finishes");
    return;
  }
  this->recording_pending_ = false;
  if (!this->mounted_ || !this->open_next_file_()) {
    this->status_momentary_error("recording_start", 5000);
    return;
  }

  this->frame_count_ = 0;
  this->bytes_written_ = 0;
  this->file_size_ = 0;
  this->max_frame_size_ = 0;
  this->first_frame_ms_ = 0;
  this->last_frame_ms_ = 0;
  this->last_flush_ms_ = millis();
  this->recording_ = true;
  this->publish_recording_(true);
  ESP_LOGI(TAG, "Recording to %s", this->filename_.c_str());
}

void CameraRecorder::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->motion_detection_enabled_ && !this->recording_)
    return;
  const uint8_t *data = image->get_data_buffer();
  const size_t length = image->get_data_length();
  if (length < 4 || data[0] != 0xFF || data[1] != 0xD8 || data[length - 2] != 0xFF || data[length - 1] != 0xD9) {
    if (this->recording_)
      this->fail_recording_("Camera output is not JPEG");
    return;
  }

  const uint32_t now = millis();
  if (this->motion_detection_enabled_ && now - this->last_motion_analysis_ms_ >= this->motion_interval_ms_) {
    if (this->queue_motion_(image))
      this->last_motion_analysis_ms_ = now;
  }

  if (!this->recording_)
    return;

  if (this->frame_count_ == 0 && !this->write_avi_header_(data, length)) {
    this->fail_recording_("Failed to write AVI header");
    return;
  }

  const uint32_t padding = length & 1;
  const uint64_t new_size = static_cast<uint64_t>(this->file_size_) + 8 + length + padding;
  const uint64_t final_size = new_size + 8 + (static_cast<uint64_t>(this->frame_count_) + 1) * 16;
  if (length > UINT32_MAX || final_size > UINT32_MAX) {
    this->fail_recording_("AVI reached the FAT32 file-size limit");
    return;
  }

  uint8_t chunk_header[8] = {'0', '0', 'd', 'c'};
  put_u32(chunk_header, 4, length);
  const uint8_t zero = 0;
  if (fwrite(chunk_header, 1, sizeof(chunk_header), this->file_) != sizeof(chunk_header) ||
      fwrite(data, 1, length, this->file_) != length ||
      (padding != 0 && fwrite(&zero, 1, 1, this->file_) != 1)) {
    this->fail_recording_("SD write failed");
    return;
  }

  uint8_t index_entry[16] = {'0', '0', 'd', 'c'};
  put_u32(index_entry, 4, 0x10);  // AVIIF_KEYFRAME
  put_u32(index_entry, 8, this->file_size_ - 220);
  put_u32(index_entry, 12, length);
  if (fwrite(index_entry, 1, sizeof(index_entry), this->index_file_) != sizeof(index_entry)) {
    this->fail_recording_("AVI index write failed");
    return;
  }

  if (this->frame_count_ == 0)
    this->first_frame_ms_ = now;
  this->last_frame_ms_ = now;
  this->frame_count_++;
  this->file_size_ = new_size;
  this->max_frame_size_ = std::max(this->max_frame_size_, static_cast<uint32_t>(length));
  this->bytes_written_ += length;
  if (millis() - this->last_flush_ms_ >= 1000) {
    if (fflush(this->file_) != 0) {
      this->fail_recording_("SD flush failed");
      return;
    }
    if (fflush(this->index_file_) != 0) {
      this->fail_recording_("AVI index flush failed");
      return;
    }
    this->last_flush_ms_ = millis();
  }
}

bool CameraRecorder::resize_motion_buffers_(size_t rgb_size, size_t pixel_count) {
  if (this->motion_rgb_size_ == rgb_size && this->motion_pixel_count_ == pixel_count)
    return true;

  heap_caps_free(this->motion_rgb_);
  heap_caps_free(this->motion_luma_history_);
  this->motion_rgb_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  this->motion_luma_history_ =
      static_cast<uint8_t *>(heap_caps_malloc(pixel_count * MOTION_HISTORY_FRAMES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->motion_rgb_ == nullptr || this->motion_luma_history_ == nullptr) {
    heap_caps_free(this->motion_rgb_);
    heap_caps_free(this->motion_luma_history_);
    this->motion_rgb_ = nullptr;
    this->motion_luma_history_ = nullptr;
    this->motion_rgb_size_ = 0;
    this->motion_pixel_count_ = 0;
    ESP_LOGE(TAG, "Failed to allocate motion buffers");
    return false;
  }

  this->motion_rgb_size_ = rgb_size;
  this->motion_pixel_count_ = pixel_count;
  this->motion_history_frames_ = 0;
  this->motion_history_slot_ = 0;
  return true;
}

bool CameraRecorder::queue_motion_(const std::shared_ptr<camera::CameraImage> &image) {
  if (this->motion_task_handle_ == nullptr || this->motion_busy_.exchange(true))
    return false;
  const uint8_t *jpeg = image->get_data_buffer();
  const size_t length = image->get_data_length();
  if (length > this->motion_jpeg_size_) {
    const size_t capacity = std::max(length, std::max(MOTION_JPEG_INITIAL_CAPACITY, this->motion_jpeg_size_ * 2));
    auto *buffer = static_cast<uint8_t *>(heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buffer == nullptr) {
      this->motion_busy_ = false;
      ESP_LOGE(TAG, "Failed to allocate %u-byte motion JPEG", static_cast<unsigned>(length));
      return false;
    }
    heap_caps_free(this->motion_jpeg_);
    this->motion_jpeg_ = buffer;
    this->motion_jpeg_size_ = capacity;
  }
  this->motion_jpeg_length_ = length;
  this->motion_source_image_ = image;
  xTaskNotifyGive(this->motion_task_handle_);
  return true;
}

void CameraRecorder::motion_task_(void *arg) {
  auto *recorder = static_cast<CameraRecorder *>(arg);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (recorder->motion_task_stop_) {
      recorder->motion_source_image_.reset();
      break;
    }
    bool copied = recorder->motion_detection_enabled_.load();
    if (copied) {
      const uint8_t *source = recorder->motion_source_image_->get_data_buffer();
      for (size_t offset = 0; offset < recorder->motion_jpeg_length_; offset += MOTION_COPY_CHUNK_SIZE) {
        if (!recorder->motion_detection_enabled_) {
          copied = false;
          break;
        }
        const size_t length = std::min(MOTION_COPY_CHUNK_SIZE, recorder->motion_jpeg_length_ - offset);
        memcpy(recorder->motion_jpeg_ + offset, source + offset, length);
        vTaskDelay(1);
      }
    }
    recorder->motion_source_image_.reset();
    float score = 0.0f;
    recorder->motion_result_valid_ = copied && recorder->motion_detection_enabled_ &&
                                     recorder->process_motion_(recorder->motion_jpeg_, recorder->motion_jpeg_length_, &score);
    recorder->motion_result_score_ = score;
    recorder->motion_result_ready_.store(true, std::memory_order_release);
    App.wake_loop_threadsafe();
  }
  recorder->motion_task_running_.store(false, std::memory_order_release);
  vTaskDelete(nullptr);
}

void CameraRecorder::publish_motion_result_() {
  if (!this->motion_result_ready_.exchange(false, std::memory_order_acquire))
    return;
  if (this->motion_detection_enabled_ && this->motion_result_valid_) {
    this->motion_score_sensor_->publish_state(this->motion_result_score_);
    if (this->motion_result_score_ >= this->motion_threshold_) {
      this->last_motion_detected_ms_ = millis();
      this->publish_motion_(true);
    }
  }
  this->motion_busy_ = false;
}

bool CameraRecorder::process_motion_(const uint8_t *jpeg, size_t length, float *score) {
  if (!this->motion_detection_enabled_)
    return false;
  uint16_t source_width;
  uint16_t source_height;
  if (length > INT_MAX || !jpeg_dimensions(jpeg, length, &source_width, &source_height))
    return false;

  // The new decoder requires 8-pixel-aligned scale dimensions.
  const uint16_t width = ((source_width + 63) / 64) * 8;
  const uint16_t height = ((source_height + 63) / 64) * 8;
  jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
  config.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
  config.scale.width = width;
  config.scale.height = height;
  jpeg_dec_handle_t decoder = nullptr;
  bool decoded = false;
  if (jpeg_dec_open(&config, &decoder) == JPEG_ERR_OK) {
    jpeg_dec_io_t io{};
    io.inbuf = const_cast<uint8_t *>(jpeg);
    io.inbuf_len = static_cast<int>(length);
    jpeg_dec_header_info_t header{};
    int output_len = 0;
    if (jpeg_dec_parse_header(decoder, &io, &header) == JPEG_ERR_OK &&
        jpeg_dec_get_outbuf_len(decoder, &output_len) == JPEG_ERR_OK && output_len > 0 &&
        static_cast<size_t>(output_len) == static_cast<size_t>(width) * height * 2 &&
        this->resize_motion_buffers_(output_len, static_cast<size_t>(width) * height)) {
      io.outbuf = this->motion_rgb_;
      decoded = jpeg_dec_process(decoder, &io) == JPEG_ERR_OK && io.out_size == output_len;
    }
    jpeg_dec_close(decoder);
  }

  if (!decoded) {
    ESP_LOGW(TAG, "Failed to decode motion JPEG");
    return false;
  }
  if (!this->motion_detection_enabled_)
    return false;
  if (this->motion_reset_pending_.exchange(false)) {
    this->motion_history_frames_ = 0;
    this->motion_history_slot_ = 0;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  uint32_t changed_recent = 0;
  uint32_t changed_over_four_frames = 0;
  const size_t history_offset = this->motion_history_slot_ * pixel_count;
  const size_t previous_offset =
      ((this->motion_history_slot_ + MOTION_HISTORY_FRAMES - 1) % MOTION_HISTORY_FRAMES) * pixel_count;
  for (size_t index = 0; index < pixel_count; index++) {
    const size_t offset = index * 2;
    const uint16_t pixel = this->motion_rgb_[offset] | (this->motion_rgb_[offset + 1] << 8);
    const uint8_t red = ((pixel >> 11) & 0x1F) * 255 / 31;
    const uint8_t green = ((pixel >> 5) & 0x3F) * 255 / 63;
    const uint8_t blue = (pixel & 0x1F) * 255 / 31;
    const uint8_t luma = (red * 77 + green * 150 + blue * 29) >> 8;
    if (this->motion_history_frames_ != 0 &&
        std::abs(static_cast<int>(luma) - this->motion_luma_history_[previous_offset + index]) >= MOTION_PIXEL_DIFFERENCE)
      changed_recent++;
    if (this->motion_history_frames_ == MOTION_HISTORY_FRAMES &&
        std::abs(static_cast<int>(luma) - this->motion_luma_history_[history_offset + index]) >= MOTION_PIXEL_DIFFERENCE)
      changed_over_four_frames++;
    this->motion_luma_history_[history_offset + index] = luma;
  }

  this->motion_history_slot_ = (this->motion_history_slot_ + 1) % MOTION_HISTORY_FRAMES;
  if (this->motion_history_frames_ < MOTION_HISTORY_FRAMES)
    this->motion_history_frames_++;
  if (this->motion_history_frames_ <= MOTION_WARMUP_FRAMES) {
    *score = 0.0f;
    return true;
  }

  *score = std::max(changed_recent, changed_over_four_frames) * 100.0f / pixel_count;
  return true;
}

bool CameraRecorder::write_avi_header_(const uint8_t *jpeg, size_t length) {
  uint16_t width;
  uint16_t height;
  if (!jpeg_dimensions(jpeg, length, &width, &height)) {
    ESP_LOGE(TAG, "Could not read JPEG dimensions");
    return false;
  }

  uint8_t header[AVI_HEADER_SIZE]{};
  memcpy(header + 0, "RIFF", 4);
  memcpy(header + 8, "AVI ", 4);
  memcpy(header + 12, "LIST", 4);
  put_u32(header, 16, 192);
  memcpy(header + 20, "hdrl", 4);
  memcpy(header + 24, "avih", 4);
  put_u32(header, 28, 56);
  put_u32(header, 32, 200000);
  put_u32(header, 56, 1);
  put_u32(header, 64, width);
  put_u32(header, 68, height);
  memcpy(header + 88, "LIST", 4);
  put_u32(header, 92, 116);
  memcpy(header + 96, "strl", 4);
  memcpy(header + 100, "strh", 4);
  put_u32(header, 104, 56);
  memcpy(header + 108, "vidsMJPG", 8);
  put_u32(header, 128, 1);
  put_u32(header, 132, 5);
  put_u32(header, 148, UINT32_MAX);
  put_u16(header, 160, width);
  put_u16(header, 162, height);
  memcpy(header + 164, "strf", 4);
  put_u32(header, 168, 40);
  put_u32(header, 172, 40);
  put_u32(header, 176, width);
  put_u32(header, 180, height);
  put_u16(header, 184, 1);
  put_u16(header, 186, 24);
  memcpy(header + 188, "MJPG", 4);
  put_u32(header, 192, static_cast<uint32_t>(width) * height * 3);
  memcpy(header + 212, "LIST", 4);
  memcpy(header + 220, "movi", 4);

  if (fwrite(header, 1, sizeof(header), this->file_) != sizeof(header))
    return false;
  this->file_size_ = sizeof(header);
  return true;
}

bool CameraRecorder::finalize_avi_() {
  if (this->frame_count_ == 0)
    return true;

  const uint32_t movi_size = this->file_size_ - 220;
  const uint32_t index_size = this->frame_count_ * 16;
  uint8_t index_header[8] = {'i', 'd', 'x', '1'};
  put_u32(index_header, 4, index_size);
  if (fflush(this->index_file_) != 0 || fseek(this->index_file_, 0, SEEK_SET) != 0 ||
      fseek(this->file_, this->file_size_, SEEK_SET) != 0 ||
      fwrite(index_header, 1, sizeof(index_header), this->file_) != sizeof(index_header))
    return false;

  uint8_t buffer[256];
  for (uint32_t remaining = index_size; remaining > 0;) {
    const size_t chunk = std::min<size_t>(sizeof(buffer), remaining);
    if (fread(buffer, 1, chunk, this->index_file_) != chunk || fwrite(buffer, 1, chunk, this->file_) != chunk)
      return false;
    remaining -= chunk;
  }
  this->file_size_ += sizeof(index_header) + index_size;

  uint32_t interval_ms = 200;
  if (this->frame_count_ > 1)
    interval_ms = std::max<uint32_t>(1, (this->last_frame_ms_ - this->first_frame_ms_) / (this->frame_count_ - 1));
  const uint32_t bytes_per_second = std::min<uint64_t>(
      UINT32_MAX,
      static_cast<uint64_t>(this->bytes_written_) * 1000 / (static_cast<uint64_t>(this->frame_count_) * interval_ms));
  const uint32_t microseconds_per_frame = std::min<uint64_t>(UINT32_MAX, static_cast<uint64_t>(interval_ms) * 1000);

  return patch_u32(this->file_, 4, this->file_size_ - 8) && patch_u32(this->file_, 32, microseconds_per_frame) &&
         patch_u32(this->file_, 36, bytes_per_second) && patch_u32(this->file_, 48, this->frame_count_) &&
         patch_u32(this->file_, 44, 0x10) &&
         patch_u32(this->file_, 60, this->max_frame_size_) && patch_u32(this->file_, 128, interval_ms) &&
         patch_u32(this->file_, 132, 1000) && patch_u32(this->file_, 140, this->frame_count_) &&
         patch_u32(this->file_, 144, this->max_frame_size_) && patch_u32(this->file_, 216, movi_size);
}

void CameraRecorder::fail_recording_(const char *message) {
  ESP_LOGE(TAG, "%s", message);
  this->status_momentary_error("recording_write", 10000);
  this->stop();
}

void CameraRecorder::stop() {
  this->recording_pending_ = false;
  this->finish_recording_(false);
}

void CameraRecorder::stop_and_classify() {
  this->recording_pending_ = false;
  this->finish_recording_(true);
}

void CameraRecorder::finish_recording_(bool classify) {
  if (!this->recording_)
    return;

  bool finalized = false;
  if (this->file_ != nullptr) {
    finalized = this->finalize_avi_();
    if (!finalized)
      ESP_LOGE(TAG, "Failed to finalize AVI header");
    if (fflush(this->file_) != 0) {
      ESP_LOGE(TAG, "Failed to flush AVI file");
      finalized = false;
    }
    if (fclose(this->file_) != 0) {
      ESP_LOGE(TAG, "Failed to close AVI file");
      finalized = false;
    }
    this->file_ = nullptr;
  }
  if (this->index_file_ != nullptr) {
    if (fclose(this->index_file_) != 0) {
      ESP_LOGE(TAG, "Failed to close AVI index");
      finalized = false;
    }
    this->index_file_ = nullptr;
    if (finalized)
      remove(this->index_filename_.c_str());
    else
      ESP_LOGE(TAG, "Retained recovery index %s", this->index_filename_.c_str());
  }
  const bool queue_classification = classify && finalized && this->frame_count_ != 0;
  if (queue_classification) {
    this->classification_queued_ms_ = millis();
    this->classification_pending_ = true;
    this->classifying_sensor_->publish_state(true);
  }
  this->recording_ = false;
  this->publish_recording_(false);
  if (this->frame_count_ == 0) {
    if (remove(this->filename_.c_str()) != 0)
      ESP_LOGE(TAG, "Could not remove empty recording %s: %s", this->filename_.c_str(), strerror(errno));
    else
      ESP_LOGI(TAG, "Discarded empty recording %s", this->filename_.c_str());
    this->update_storage_();
    if (this->auto_deletion_)
      this->auto_delete_pending_ = true;
    return;
  }
  ESP_LOGI(TAG, "Saved %s (%lu frames, %zu bytes)", this->filename_.c_str(),
           static_cast<unsigned long>(this->frame_count_), this->bytes_written_);

  if (classify && !finalized) {
    ESP_LOGW(TAG, "Classification skipped because the AVI was not finalized");
  }
  this->update_storage_();
  if (!queue_classification && this->auto_deletion_)
    this->auto_delete_pending_ = true;
}

bool CameraRecorder::models_ready_() const {
  struct ModelFile {
    const char *name;
    off_t size;
  };
  static const ModelFile MODELS[] = {
      {PERSON_MODEL, 435328},
  };
  struct stat info {};
  for (const auto &model : MODELS) {
    const std::string path = std::string(MODEL_DIRECTORY) + "/" + model.name;
    if (stat(path.c_str(), &info) != 0 || info.st_size != model.size) {
      ESP_LOGE(TAG, "Missing or invalid model: %s", path.c_str());
      return false;
    }
  }
  return true;
}

struct ClassificationTiming {
  uint32_t read_ms;
  uint32_t decode_ms;
  uint32_t inference_ms;
};

static bool classify_avi_frame(FILE *file, uint32_t index_offset, uint32_t frame_index, uint32_t max_frame_size,
                               PedestrianDetect &detector, bool *person, ClassificationTiming *timing) {
  const uint32_t read_start = millis();
  const uint64_t entry_offset = static_cast<uint64_t>(index_offset) + 8 + static_cast<uint64_t>(frame_index) * 16;
  if (entry_offset > LONG_MAX || fseek(file, static_cast<long>(entry_offset), SEEK_SET) != 0)
    return false;

  uint8_t entry[16];
  if (fread(entry, 1, sizeof(entry), file) != sizeof(entry) || memcmp(entry, "00dc", 4) != 0)
    return false;
  const uint32_t jpeg_offset = get_u32(entry, 8);
  const uint32_t jpeg_size = get_u32(entry, 12);
  const uint64_t chunk_offset = 220ULL + jpeg_offset;
  if (jpeg_size < 4 || jpeg_size > max_frame_size || chunk_offset + 8 + jpeg_size > index_offset ||
      chunk_offset > LONG_MAX || fseek(file, static_cast<long>(chunk_offset), SEEK_SET) != 0)
    return false;

  uint8_t chunk_header[8];
  if (fread(chunk_header, 1, sizeof(chunk_header), file) != sizeof(chunk_header) ||
      memcmp(chunk_header, "00dc", 4) != 0 || get_u32(chunk_header, 4) != jpeg_size)
    return false;

  auto *jpeg = static_cast<uint8_t *>(heap_caps_malloc(jpeg_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (jpeg == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u-byte classification JPEG", static_cast<unsigned>(jpeg_size));
    return false;
  }
  const bool read = fread(jpeg, 1, jpeg_size, file) == jpeg_size;
  if (!read || jpeg[0] != 0xFF || jpeg[1] != 0xD8 || jpeg[jpeg_size - 2] != 0xFF || jpeg[jpeg_size - 1] != 0xD9) {
    heap_caps_free(jpeg);
    return false;
  }

  const uint32_t read_end = millis();
  auto image = decode_classification_frame(jpeg, jpeg_size);
  heap_caps_free(jpeg);
  if (image.data == nullptr)
    return false;
  const uint32_t decode_end = millis();
  *person = !detector.run(image).empty();
  const uint32_t inference_end = millis();
  heap_caps_free(image.data);
  timing->read_ms = read_end - read_start;
  timing->decode_ms = decode_end - read_end;
  timing->inference_ms = inference_end - decode_end;
  return true;
}

void CameraRecorder::run_classification_() {
  if (this->classification_state_ == nullptr) {
    const uint32_t setup_start = millis();
    if (this->detector_ == nullptr) {
      ESP_LOGE(TAG, "Pedestrian model is not loaded");
      this->finish_classification_(false, false);
      return;
    }

    auto *state = new (std::nothrow) ClassificationState;
    if (state == nullptr) {
      ESP_LOGE(TAG, "Could not allocate classification state");
      this->finish_classification_(false, false);
      return;
    }
    state->file = fopen(this->filename_.c_str(), "rb");
    if (state->file == nullptr) {
      ESP_LOGE(TAG, "Could not open recording for classification: %s", strerror(errno));
      delete state;
      this->finish_classification_(false, false);
      return;
    }
    const uint64_t index_size = static_cast<uint64_t>(this->frame_count_) * 16;
    if (index_size + 8 > this->file_size_) {
      fclose(state->file);
      state->file = nullptr;
      delete state;
      ESP_LOGE(TAG, "AVI index is invalid");
      this->finish_classification_(false, false);
      return;
    }
    state->index_offset = this->file_size_ - index_size - 8;
    uint8_t index_header[8];
    if (state->index_offset > LONG_MAX || fseek(state->file, state->index_offset, SEEK_SET) != 0 ||
        fread(index_header, 1, sizeof(index_header), state->file) != sizeof(index_header) ||
        memcmp(index_header, "idx1", 4) != 0 || get_u32(index_header, 4) != index_size) {
      fclose(state->file);
      state->file = nullptr;
      delete state;
      ESP_LOGE(TAG, "Could not read AVI index for classification");
      this->finish_classification_(false, false);
      return;
    }
    for (uint8_t marker = 0; marker < 4; marker++) {
      const uint32_t frame = static_cast<uint64_t>(this->frame_count_) * marker / 4;
      if (state->sample_count == 0 || state->samples[state->sample_count - 1] != frame)
        state->samples[state->sample_count++] = frame;
    }
    this->classification_state_ = state;
    ESP_LOGI(TAG, "Classifying %u frames; %u detections required (queue wait %u ms, AVI setup %u ms)",
             static_cast<unsigned>(state->sample_count), static_cast<unsigned>(this->min_person_detections_),
             static_cast<unsigned>(setup_start - this->classification_queued_ms_),
             static_cast<unsigned>(millis() - setup_start));
    state->last_sample_end_ms = millis();
    if (state->sample_count < this->min_person_detections_) {
      this->finish_classification_(true, false);
      return;
    }
  }

  auto *state = this->classification_state_;
  const uint32_t frame = state->samples[state->next_sample];
  const uint32_t sample_start = millis();
  bool detected = false;
  ClassificationTiming timing{};
  if (!classify_avi_frame(state->file, state->index_offset, frame, this->max_frame_size_, *this->detector_, &detected,
                          &timing)) {
    ESP_LOGE(TAG, "Failed to classify AVI frame %u", static_cast<unsigned>(frame));
    this->finish_classification_(false, false);
    return;
  }
  const uint32_t sample_end = millis();
  ESP_LOGI(TAG, "Classification frame %u timing: loop wait %u ms, SD read %u ms, JPEG decode %u ms, inference %u ms, total %u ms",
           static_cast<unsigned>(frame), static_cast<unsigned>(sample_start - state->last_sample_end_ms),
           static_cast<unsigned>(timing.read_ms), static_cast<unsigned>(timing.decode_ms),
           static_cast<unsigned>(timing.inference_ms), static_cast<unsigned>(sample_end - sample_start));
  state->last_sample_end_ms = millis();
  state->next_sample++;
  if (detected)
    state->detections++;
  ESP_LOGD(TAG, "Classification frame %u: %s (%u/%u detections)", static_cast<unsigned>(frame),
           detected ? "person" : "no person", static_cast<unsigned>(state->detections),
           static_cast<unsigned>(this->min_person_detections_));

  const uint8_t remaining = state->sample_count - state->next_sample;
  if (state->detections >= this->min_person_detections_)
    this->finish_classification_(true, true);
  else if (state->detections + remaining < this->min_person_detections_)
    this->finish_classification_(true, false);
}

void CameraRecorder::finish_classification_(bool succeeded, bool person) {
  this->clear_classification_state_();
  this->person_detected_sensor_->publish_state(person);

  const char *label = person ? "PERSON" : (succeeded ? "MOTION" : "UNCLASSIFIED");
  const bool keep = !succeeded || (person ? this->record_people_ : this->record_other_motion_);
  if (keep) {
    const std::string suffix = std::string("_") + label + ".avi";
    const std::string target = this->unique_filename_(this->filename_stem_, suffix.c_str());
    if (target.empty() || rename(this->filename_.c_str(), target.c_str()) != 0) {
      ESP_LOGE(TAG, "Could not label %s: %s", this->filename_.c_str(), strerror(errno));
    } else {
      this->filename_ = target;
      ESP_LOGI(TAG, "Classified recording as %s", this->filename_.c_str());
    }
  } else if (remove(this->filename_.c_str()) != 0) {
    ESP_LOGE(TAG, "Could not remove filtered recording %s: %s", this->filename_.c_str(), strerror(errno));
  } else {
    ESP_LOGI(TAG, "Removed recording rejected by classification switches");
  }

  this->classifying_sensor_->publish_state(false);
  this->update_storage_();
  ESP_LOGI(TAG, "Classification finished in %u ms", static_cast<unsigned>(millis() - this->classification_queued_ms_));
  this->classification_pending_ = false;
  if (this->auto_deletion_)
    this->auto_delete_pending_ = true;
}

void CameraRecorder::clear_classification_state_() {
  if (this->classification_state_ == nullptr)
    return;
  if (this->classification_state_->file != nullptr)
    fclose(this->classification_state_->file);
  delete this->classification_state_;
  this->classification_state_ = nullptr;
}

void CameraRecorder::publish_recording_(bool state) {
  if (this->recording_sensor_ != nullptr)
    this->recording_sensor_->publish_state(state);
}

void CameraRecorder::publish_motion_(bool state) {
  if (this->motion_active_ == state)
    return;
  this->motion_active_ = state;
  this->motion_sensor_->publish_state(state);
}

void CameraRecorder::on_shutdown() {
  if (this->prune_dir_ != nullptr) {
    closedir(this->prune_dir_);
    this->prune_dir_ = nullptr;
  }
  this->stop();
  this->clear_classification_state_();
  if (this->motion_task_handle_ != nullptr) {
    this->motion_task_stop_ = true;
    xTaskNotifyGive(this->motion_task_handle_);
    const uint32_t started = millis();
    while (this->motion_task_running_.load(std::memory_order_acquire) && millis() - started < 2000)
      vTaskDelay(1);
    if (!this->motion_task_running_)
      this->motion_task_handle_ = nullptr;
  }
  delete this->detector_;
  this->detector_ = nullptr;
  if (!this->motion_task_running_) {
    heap_caps_free(this->motion_jpeg_);
    heap_caps_free(this->motion_rgb_);
    heap_caps_free(this->motion_luma_history_);
    this->motion_jpeg_ = nullptr;
    this->motion_rgb_ = nullptr;
    this->motion_luma_history_ = nullptr;
  }
  if (this->mounted_) {
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, this->card_);
    this->mounted_ = false;
  }
  if (this->web_server_ != nullptr)
    this->web_server_->deinit();
}

}  // namespace esphome::camera_recorder
