# yahboom_web_savmap_interfaces

Pacote de interfaces ROS 2 para o sistema de guardar mapas da app Yahboom.

## Serviço

`WebSaveMap.srv`:
```
string mapname    # nome do mapa a guardar
---
string response   # resposta do servidor
```

Usado exclusivamente por `yahboom_app_save_map`.
