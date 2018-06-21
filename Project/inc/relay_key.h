#ifndef __RELAY_KEY_H
#define __RELAY_KEY_H

void relay_key_init();
void relay_restore_keystates();
bool relay_get_key(u8 _key);
bool relay_set_key(u8 _key, bool _on);
bool IsValidRelaykey(u8 index);
void relay_set_keys(uint8_t keyvalue);

#endif /* __RELAY_KEY_H */