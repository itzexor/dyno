#include "link.h"
#include <string.h>

void dyno_link_init(struct dyno_link *link, struct dyno_core *core) {
    memset(link, 0, sizeof(*link));
    link->core = core;
    dyno_request(core, (const uint8_t *)"", 0, link->reply);
}
void dyno_link_write(struct dyno_link *link, const uint8_t *data, size_t size) {
    if (size == 2 && data[0] == 0xf0 && data[1] < 4) {
        link->page = data[1]; link->paged = true; return;
    }
    link->paged = false; link->page = 0;
    dyno_request(link->core, data, size, link->reply);
}
uint8_t dyno_link_byte(const struct dyno_link *link, size_t index) {
    size_t limit = link->paged ? 32u : dyno_response_size(link->reply[3]);
    if (index >= limit) return 0xff;
    return link->reply[(link->paged ? (size_t)link->page * 32u : 0u) + index];
}
