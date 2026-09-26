#include "PoeStatusServer.h"

PoeStatusServer PoeStatus;

#if defined(BOARD_ATOMS3_POE)

#include <M5_Ethernet.h>
#include <WiFi.h>

#include <cstring>

#include "FirmwareVersion.h"
#include "NetworkManager.h"
#include "rtslogo.h"
#include "rtsnow_node.h"

namespace
{
EthernetServer ethServer(80);
WiFiServer wifiServer(80);

// Ethernet (W5500's own hardware TCP/IP stack) and WiFi (ESP32's native
// lwIP) are two fully independent transports, each able to have an
// unrelated client mid-request at the same time - one request in flight
// per transport, a human occasionally loading a diagnostic page, not a
// production multi-client web server.
template <typename ClientT>
struct ListenerState
{
    ClientT client;
    char requestLine[160] = {0}; // only the first line ("GET /path HTTP/1.1") ever matters - see below
    size_t requestLineLen = 0;
    bool requestLineDone = false;
    uint32_t clientStartMs = 0;

    void reset()
    {
        requestLineLen = 0;
        requestLineDone = false;
    }
};

ListenerState<EthernetClient> ethState;
ListenerState<WiFiClient> wifiState;

// A client that connects but never finishes sending even its first line
// (dead browser tab, port scanner) gets forcibly dropped after this long,
// so one stuck connection can't wedge this feature - pollListener() must
// always return promptly either way, never block waiting for a slow or
// absent client.
constexpr uint32_t kClientTimeoutMs = 3000;

template <typename ClientT>
void printMac(ClientT &client, const uint8_t mac[6])
{
    client.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Shared page chrome (logo/title header, left nav, section box styling) -
// visually modeled after a Digi XBee gateway's own admin UI (dark-blue
// section bars, light-blue sub-headers, left sidebar) since that's a
// layout language anyone who's used one of those will recognize
// immediately, without pulling in any actual framework/JS.
template <typename ClientT>
void writeStyle(ClientT &client)
{
    client.println("<style>"
                    "body{font-family:-apple-system,Arial,sans-serif;margin:0;background:#fff;color:#222;}"
                    "#topbar{display:flex;align-items:center;gap:16px;padding:12px 20px;}"
                    "#topbar img{width:100px;height:100px;}"
                    "#topbar h1{font-size:1.4em;margin:0;color:#001a70;}"
                    "#layout{display:flex;}"
                    "#sidebar{width:170px;flex-shrink:0;padding:16px 12px;border-right:1px solid #ddd;}"
                    "#sidebar a{display:block;color:#0645ad;text-decoration:none;font-size:0.92em;margin-bottom:8px;}"
                    "#sidebar a:hover{text-decoration:underline;}"
                    "#sidebar .navhead{font-weight:700;color:#222;margin:14px 0 4px;font-size:0.9em;}"
                    "#main{flex:1;padding:16px 20px;min-width:0;}"
                    ".box{border:1px solid #b9c9e8;margin-bottom:14px;border-radius:2px;overflow:hidden;}"
                    ".box h2{background:#001a70;color:#fff;font-size:1em;margin:0;padding:8px 12px;}"
                    ".box h3{background:#dbe6fb;color:#001a70;font-size:0.92em;margin:0;padding:6px 12px;}"
                    ".box .body{padding:10px 14px;}"
                    ".box .body p{margin:4px 0;font-size:0.92em;}"
                    "table{border-collapse:collapse;width:100%;font-size:0.9em;}"
                    "th{text-align:left;color:#001a70;border-bottom:2px solid #b9c9e8;padding:5px 8px;}"
                    "td{border-bottom:1px solid #eee;padding:5px 8px;}"
                    "td.rssi{font-family:monospace;letter-spacing:1px;}"
                    "</style>");
}

// Polls /api/status (see writeStatusJsonResponse()) every 5s and patches
// only the dynamic containers in place - no <meta refresh>/full page
// reload, so the page no longer blinks on every update. Written to build
// HTML from raw JSON values on the client side (rather than the server
// sending pre-built HTML fragments) so there's exactly one place
// (this function) that knows how to render the dynamic sections, not two
// copies that could drift - the very first render on page load reuses
// this same fetch, so the C++ side never needs to pre-render these
// sections into the initial HTML at all.
template <typename ClientT>
void writeScript(ClientT &client)
{
    client.println("<script>"
                    // Same 4-bar glyph + dBm thresholds as the desktop app's
                    // rssi_bar_count()/format_rssi() (node_network_page.py) -
                    // one shared visual language for "what counts as a good
                    // link" across both surfaces, not two independently
                    // tuned scales.
                    "function rssiBars(rssi){"
                    "if(rssi===null||rssi===undefined)return '–';"
                    "var t=[-50,-60,-70,-80],f=['▂','▄','▆','█'],bars=0;"
                    "for(var i=0;i<t.length;i++)if(rssi>=t[i])bars++;"
                    "var g='';"
                    "for(var i=0;i<4;i++)g+=(i<bars?f[i]:'▁');"
                    "return g+' '+rssi+' dBm';"
                    "}"
                    "function formatUptime(s){"
                    "var d=Math.floor(s/86400);s%=86400;"
                    "var h=Math.floor(s/3600);s%=3600;"
                    "var m=Math.floor(s/60);s%=60;"
                    "var parts=[];"
                    "if(d>0)parts.push(d+'d');"
                    "if(d>0||h>0)parts.push(h+'h');"
                    "if(d>0||h>0||m>0)parts.push(m+'m');"
                    "parts.push(s+'s');"
                    "return parts.join(' ');"
                    "}"
                    "function refresh(){"
                    "fetch('/api/status').then(function(r){return r.json();}).then(function(d){"
                    "document.getElementById('uptime').textContent=formatUptime(d.uptime);"
                    "document.getElementById('freeHeap').textContent=d.freeHeap+' bytes';"
                    "document.getElementById('connStatus').textContent=d.connStatus;"
                    // Only ever one of these two now (firmware only tries
                    // WiFi at all when Ethernet didn't come up) - showing
                    // just the active transport, not an explanation of the
                    // inactive one (that detail still lives in the
                    // Diagnostics box below for Ethernet specifically).
                    "var net='';"
                    "if(d.ethIp)net+='<p>Ethernet IP: '+d.ethIp+'</p>';"
                    "if(d.wifiIp)net+='<p>WiFi IP: '+d.wifiIp+' (SSID: '+d.wifiSsid+', '+rssiBars(d.wifiRssi)+')</p>';"
                    "document.getElementById('netDetails').innerHTML=net;"
                    "var rows='';"
                    "if(d.peers.length===0)rows='<tr><td colspan=\"4\">No mesh devices heard yet</td></tr>';"
                    "d.peers.forEach(function(p){"
                    "rows+='<tr><td>0x'+p.id+'</td><td>'+p.mac+'</td><td class=\"rssi\">'+rssiBars(p.rssi)+'</td><td>'+"
                    "(p.ago<0?'never':p.ago+'s ago')+'</td></tr>';"
                    "});"
                    "document.getElementById('meshBody').innerHTML=rows;"
                    "}).catch(function(){});"
                    "}"
                    "refresh();"
                    "setInterval(refresh,5000);"
                    "</script>");
}

template <typename ClientT>
void writeHeader(ClientT &client, const char *title)
{
    client.println("<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
                    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.printf("<title>%s</title>", title);
    writeStyle(client);
    client.println("</head><body>");
    client.println("<div id=\"topbar\"><img src=\"/logo.gif\" alt=\"RTS\">"
                    "<h1>RTS-NOW Gateway Status</h1></div>");
    client.println("<div id=\"layout\"><div id=\"sidebar\">"
                    "<div class=\"navhead\">On this page</div>"
                    "<a href=\"#device\">Device</a>"
                    "<a href=\"#network\">Network</a>"
                    "<a href=\"#diagnostics\">Diagnostics</a>"
                    "<a href=\"#mesh\">Mesh Devices</a>"
                    "</div><div id=\"main\">");
}

// Every request gets the exact same page regardless of path/method/which
// transport it arrived on - this is a single-purpose diagnostic surface,
// not a real router. Only the truly static-after-boot diagnostics are
// server-rendered directly here; Device/Network/Mesh Devices are empty
// placeholders filled in by writeScript()'s fetch loop (see its own
// comment) so they can update without a page reload.
template <typename ClientT>
void writeResponse(ClientT &client)
{
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    writeHeader(client, "RTS-NOW POE Status");

    client.println("<div class=\"box\" id=\"device\"><h2>Device</h2><div class=\"body\">");
    client.printf("<p><b>%s</b></p>\n", rtsnowNodeFriendlyName());
    client.printf("<p>MAC address: %s</p>\n", WiFi.macAddress().c_str());
    client.println("<p>Uptime: <span id=\"uptime\">-</span></p>");
    client.println("<p>Free heap: <span id=\"freeHeap\">-</span></p>");
    client.printf("<p>Firmware: %u.%u.%u</p>\n", FirmwareVersion::kMajor, FirmwareVersion::kMinor,
                  FirmwareVersion::kPatch);
    client.println("</div></div>");

    client.println("<div class=\"box\" id=\"network\"><h2>Network</h2><div class=\"body\">");
    client.println("<p>Connection status: <b id=\"connStatus\">-</b></p>");
    client.println("<div id=\"netDetails\"></div>");
    client.println("</div></div>");

    client.println("<div class=\"box\" id=\"diagnostics\"><h3>PoE Base / W5500 diagnostics</h3><div class=\"body\">");
    client.printf("<p>PoE Base (W5500): %s</p>\n", Network.hardwareDetected() ? "Detected" : "NOT detected");
    // Raw SPI register read, independent of the line above - see
    // NetworkManager's own rawReadW5500Version() comment. 0x04 is the
    // W5500's fixed, hardcoded chip ID; anything else means nothing
    // is answering on the SPI bus at all, regardless of what the
    // M5_Ethernet library's own check (the line above) concluded.
    client.printf("<p>Raw SPI VERSIONR read: 0x%02X (expected 0x04)</p>\n", Network.lastRawVersionRead());
    client.printf("<p>Raw SPI write test: %s</p>\n",
                  Network.lastRawWriteWorked() ? "writes ARE taking effect" : "writes NOT taking effect");
    client.printf("<p>Raw soft-reset test: MR=0x%02X after %lums (%s)</p>\n", Network.lastSoftResetFinalMr(),
                  static_cast<unsigned long>(Network.lastSoftResetElapsedMs()),
                  Network.lastSoftResetFinalMr() == 0x00 ? "reset completed" : "reset did NOT complete in time");
    // Byte-for-byte replica of isW5500() itself, run as one continuous
    // SPI transaction (matching the library's own, unlike the tests
    // above which each open/close their own transaction) - if this
    // passes, the library SHOULD have detected the chip too, pointing at
    // something in Ethernet.init()'s own call path beyond what's
    // replicated here (chip=52 W5200 probe state, CS pin setup timing,
    // etc.) rather than the SPI protocol sequence itself.
    {
        const char *step = "-";
        switch (Network.lastIsW5500ReplicaResult())
        {
        case 0: step = "PASSED - should have been detected"; break;
        case 1: step = "FAILED at softReset"; break;
        case 2: step = "FAILED at MR=0x08 verify"; break;
        case 3: step = "FAILED at MR=0x10 verify"; break;
        case 4: step = "FAILED at MR=0x00 verify"; break;
        case 5: step = "FAILED at final VERSIONR check"; break;
        }
        client.printf("<p>isW5500() exact replica: %s</p>\n", step);
    }
    client.println("</div></div>");

    client.println("<div class=\"box\" id=\"mesh\"><h2>Mesh Devices</h2><div class=\"body\">");
    client.println("<table><tr><th>Device ID</th><th>MAC</th>"
                    "<th>RSSI</th><th>Last heard</th></tr><tbody id=\"meshBody\"></tbody></table>");
    client.println("</div></div>");

    client.println("</div></div>");
    writeScript(client);
    client.println("</body></html>");
}

// Raw data for the dynamic sections above, as JSON - see writeScript()'s
// own comment for why the client builds HTML from this rather than the
// server sending HTML fragments directly. Hand-rolled (no ArduinoJson
// dependency) since both ends of this format are written by this same
// file - there's no external consumer to stay compatible with.
template <typename ClientT>
void writeStatusJsonResponse(ClientT &client)
{
    RTSNowPeerSnapshot peers[RTSNOW_NODE_MAX_PEERS];
    uint8_t peerCount = rtsnowNodePeers(peers, RTSNOW_NODE_MAX_PEERS);
    uint32_t now = millis();
    bool wifiConnected = WiFi.status() == WL_CONNECTED;

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();

    client.printf("{\"uptime\":%lu,\"freeHeap\":%u,", static_cast<unsigned long>(now / 1000), ESP.getFreeHeap());

    // Ethernet takes priority when both are up, matching
    // ipv4AddressProvider's own preference in main_atom_node.cpp.
    if (Network.isConnected())
        client.print("\"connStatus\":\"Ethernet\",");
    else if (wifiConnected)
        client.print("\"connStatus\":\"WiFi (fallback)\",");
    else
        client.print("\"connStatus\":\"Disconnected\",");

    if (Network.isConnected())
        client.printf("\"ethIp\":\"%s\",", Network.localIP().toString().c_str());
    else
        // Only reachable at all right now because of the WiFi fallback -
        // this is the whole point of that fallback existing: seeing WHY
        // Ethernet failed without needing USB/serial access, which is
        // otherwise the only way to read NetworkManager.cpp's own staged
        // Serial.printf diagnostics.
        client.printf("\"ethError\":\"%s\",", Network.lastDiagnostic());

    // Ethernet takes priority in the JSON too, not just connStatus above -
    // WiFi is only ever meant to be up when Ethernet isn't (see
    // main_atom_node.cpp's boot sequence, which only calls WiFi.begin() at
    // all if Ethernet failed), but reporting it unconditionally here (as
    // before) meant firmware that predates that guard, or somehow ends up
    // with both associated at once, kept showing wireless details the user
    // explicitly asked to be hidden whenever Ethernet is the active link.
    if (wifiConnected && !Network.isConnected())
        client.printf("\"wifiIp\":\"%s\",\"wifiSsid\":\"%s\",\"wifiRssi\":%d,", WiFi.localIP().toString().c_str(),
                      WiFi.SSID().c_str(), WiFi.RSSI());

    client.print("\"peers\":[");
    for (uint8_t i = 0; i < peerCount; i++)
    {
        const RTSNowPeerSnapshot &p = peers[i];
        if (i > 0)
            client.print(",");
        client.print("{\"id\":\"");
        client.print(p.deviceID, HEX);
        client.print("\",\"mac\":\"");
        printMac(client, p.mac);
        client.print("\",\"rssi\":");
        if (p.rssi == RTSNOW_RSSI_UNKNOWN)
            client.print("null");
        else
            client.print(p.rssi);
        client.print(",\"ago\":");
        if (p.rssi == RTSNOW_RSSI_UNKNOWN)
            client.print(-1);
        else
            client.print(static_cast<unsigned long>((now - p.lastSeenMs) / 1000));
        client.print("}");
    }
    client.println("]}");
}

// Chunk size for the logo body write below - deliberately small and
// fixed, well under the W5500's per-socket TX buffer (M5_Ethernet's
// W5100.SSIZE, commonly 2048 bytes with 8 sockets sharing 16KB). A real
// bug in that library (socket.cpp's socketSend(): when a single
// EthernetClient::write() call exceeds SSIZE, it silently clamps to
// SSIZE bytes actually sent but the caller still reports the FULL
// requested size as successfully written) meant a one-shot
// client.write(entire logo) truncated over Ethernet with no error -
// working fine over WiFi purely because WiFiClient's underlying native
// lwIP stack has no equivalent per-call limit. Writing in small chunks,
// checking each chunk's own actual return value, sidesteps the bug
// entirely rather than depending on a library fix.
constexpr size_t kLogoChunkSize = 512;

// Raw binary response, no path/method awareness of its own - only ever
// called for a "/logo.gif" request (see pollListener()'s routing below).
template <typename ClientT>
void writeLogoResponse(ClientT &client)
{
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: image/gif");
    client.printf("Content-Length: %u\n", rts_gif_len);
    client.println("Connection: close");
    client.println();
    size_t sent = 0;
    while (sent < rts_gif_len)
    {
        size_t remaining = rts_gif_len - sent;
        size_t chunk = remaining < kLogoChunkSize ? remaining : kLogoChunkSize;
        size_t written = client.write(rts_gif + sent, chunk);
        if (written == 0)
            break;  // client disconnected mid-transfer - nothing more we can do
        sent += written;
    }
}

// Pulls just the path out of a request line ("GET /logo.gif HTTP/1.1" ->
// "/logo.gif") - the only piece pollListener()'s routing needs. Leaves
// outPath empty (routes to the default status page) if the line doesn't
// look like a well-formed request line at all, rather than guessing.
void extractPath(const char *requestLine, char *outPath, size_t outSize)
{
    outPath[0] = '\0';
    const char *pathStart = strchr(requestLine, ' ');
    if (pathStart == nullptr)
        return;
    pathStart++;
    const char *pathEnd = strchr(pathStart, ' ');
    if (pathEnd == nullptr)
        return;
    size_t len = static_cast<size_t>(pathEnd - pathStart);
    if (len >= outSize)
        len = outSize - 1;
    memcpy(outPath, pathStart, len);
    outPath[len] = '\0';
}

// Shared accept/read/timeout/respond state machine for one transport's
// listener - never blocks: reads whatever's arrived so far, and if the
// client hasn't finished sending its request line yet THIS tick, just
// returns; the next call picks up where this left off.
template <typename ServerT, typename ClientT>
void pollListener(ServerT &server, ListenerState<ClientT> &st, const char *label)
{
    if (!st.client || !st.client.connected())
    {
        st.client = server.available();
        if (!st.client)
            return;
        st.reset();
        st.clientStartMs = millis();
    }

    while (st.client.available())
    {
        char c = st.client.read();
        if (c == '\n')
        {
            st.requestLineDone = true; // only the first line is ever needed - headers/body (if any) are simply never read
            break;
        }
        if (c != '\r' && st.requestLineLen < sizeof(st.requestLine) - 1)
            st.requestLine[st.requestLineLen++] = c;
    }

    if (st.requestLineDone)
    {
        st.requestLine[st.requestLineLen] = '\0';
        Serial.printf("PoeStatusServer (%s): %s\n", label, st.requestLine);
        char path[32];
        extractPath(st.requestLine, path, sizeof(path));
        if (strcmp(path, "/logo.gif") == 0)
            writeLogoResponse(st.client);
        else if (strcmp(path, "/api/status") == 0)
            writeStatusJsonResponse(st.client);
        else
            writeResponse(st.client);
        st.client.stop();
        st.reset();
        return;
    }

    if (millis() - st.clientStartMs > kClientTimeoutMs)
    {
        Serial.printf("PoeStatusServer (%s): client timed out before sending a request line - dropping\n", label);
        st.client.stop();
        st.reset();
    }
}
} // namespace

void PoeStatusServer::begin()
{
    // Each listener only ever starts if its transport is actually
    // connected at this moment - neither Ethernet nor the WiFi fallback
    // reconnect after an initial failure in this design (see their own
    // setup() comments in main_atom_node.cpp), so this is a one-shot,
    // boot-time decision, not something re-checked later.
    if (Network.isConnected())
    {
        ethServer.begin();
        Serial.printf("PoeStatusServer: Ethernet listening on http://%s/\n", Network.localIP().toString().c_str());
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        wifiServer.begin();
        Serial.printf("PoeStatusServer: WiFi listening on http://%s/\n", WiFi.localIP().toString().c_str());
    }
}

void PoeStatusServer::handleClient()
{
    // Polling a never-started/disconnected transport's server is
    // harmless (always empty) - simpler than tracking which one(s)
    // begin() actually started.
    pollListener(ethServer, ethState, "eth");
    pollListener(wifiServer, wifiState, "wifi");
}

#else

// Only meaningful on the AtomS3+PoE Ethernet board (see this file's own
// top #if). main_atom_node.cpp includes PoeStatusServer.h unconditionally
// (like DryerWebServer.h) but only ever calls these under its own
// BOARD_ATOMS3_POE guard - stubs kept here (rather than left undefined)
// so a future accidental unguarded call site is a silent no-op, not a
// link error.
void PoeStatusServer::begin() {}
void PoeStatusServer::handleClient() {}

#endif
