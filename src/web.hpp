#include <string>
#include <functional>

typedef std::function<std::string(uint8_t *, size_t)> MessageCallbackType;

void StartWebServer(MessageCallbackType onMessage);
