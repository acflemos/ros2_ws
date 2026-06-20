# laserscan_to_point_pulisher

Converte dados LaserScan em `nav_msgs/Path` (lista de PoseStamped com as posições dos pontos do scan).

## Tópicos

| Direção | Nome | Tipo |
|---------|------|------|
| Sub | `/scan` | `sensor_msgs/LaserScan` |
| Pub | `/scan_points` | `nav_msgs/Path` |

## Funcionamento

Para cada ponto do LaserScan calcula:
```
x = range * cos(angle_min + i * angle_increment)
y = range * sin(angle_min + i * angle_increment)
```
e publica como `PoseStamped` dentro de um `Path`.

Útil para visualizar o alcance do LiDAR como uma sequência de poses em RViz.

**Nota:** typo no nome do pacote (`pulisher` em vez de `publisher`) — mantido para compatibilidade.
