/*
 * webserver.h
 */

#ifndef WEBUI_H_
#define WEBUI_H_

// call once from setup() to initialize the WebUI component
bool WebUI_setup();

// call every time the balancing values are updated (dt is in Hertz)
void WebUI_loop(int dt);

#endif /* WEBUI_H_ */
