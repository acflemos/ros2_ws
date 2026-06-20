# yahboom_app_save_map

Servidor ROS 2 que guarda mapas de navegação Nav2 em base de dados SQLite, para a app móvel Yahboom.

## Nós

| Executável | Função |
|-----------|--------|
| `server` | Serviço que guarda o mapa atual |
| `client` | Cliente de teste |

## Serviço

`yahboomAppSaveMap` (tipo: `yahboom_web_savmap_interfaces/WebSaveMap`)

```
Request:  string mapname    # nome para o mapa
Response: string response   # "success" ou mensagem de erro
```

## Funcionamento (server)

1. Recebe pedido com nome do mapa
2. Cria entrada em SQLite (`/home/yahboom/cartoros2/data/xgo.db`) com ID único (MD5)
3. Chama `nav2_map_server/map_saver_cli` para guardar ficheiro PGM+YAML em `/home/yahboom/cartoros2/data/maps/`
4. Retorna confirmação

**Caminhos hardcoded** para `/home/yahboom/cartoros2/` — falha em qualquer outra máquina.
