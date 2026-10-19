# Zumo 2040: Sistemas Embebidos Avanzados

Prácticas en C con el robot **Pololu Zumo 2040** (RP2040) y la **Raspberry Pi Pico 2 / 2 W** (RP2350), usando el Pico SDK.

| Carpeta | Placa | Qué hace |
|---|---|---|
| `Blink/` | Zumo 2040 | Parpadea el LED de GP25 (prueba del entorno) |
| `MotorTests/` | Zumo 2040 | Pruebas de los motores con la librería de Pololu |
| `CANbus/PicoCAN/` | Zumo 2040 | Nodo CAN con MCP2515. Envía el ID `0x500` |
| `CANbus/Pico2wCAN/` | Pico 2 / 2 W | Nodo CAN con MCP2515. Envía el ID `0x200` |
| `zumo-2040-robot/` | — | Librería oficial de Pololu (submódulo) |

## Clonar

La librería de Pololu es un submódulo, así que se clona con `--recursive`:

```bash
git clone --recursive https://github.com/alb3r7g12/zumo2040-sistemas-embebidos.git
```

Si ya lo clonaste sin esa opción: `git submodule update --init`.

## Compilar y cargar

1. Instala la extensión **Raspberry Pi Pico** de VS Code. Descarga el SDK 2.3.1 y el toolchain la primera vez.
2. Abre **una sola carpeta de proyecto** (por ejemplo `CANbus/PicoCAN`), no la raíz del repo.
3. Pon la placa en modo BOOTSEL:
   - **Zumo:** mantén **B**, presiona y suelta **Reset**, luego suelta **B**.
   - **Pico:** mantén **BOOTSEL** mientras conectas el USB.
4. Presiona **Run** en la barra inferior.
5. Para ver la salida: `minicom -D /dev/ttyACM0` (sal con Ctrl+A y luego X).

## CAN bus (MCP2515 + TJA1050)

Bus CAN a 500 kbps, con módulos MCP2515 de cristal de 8 MHz. Al iniciar, cada nodo hace una prueba de loopback y luego pasa a modo normal. Ahí envía su ID cada segundo e imprime lo que recibe del otro nodo.

| MCP2515 | Zumo 2040 | Pico 2 (pin físico) |
|---|---|---|
| SO (MISO) | GP0 | GP16 (21) |
| CS | GP1 | GP17 (22) |
| SCK | GP2 | GP18 (24) |
| SI (MOSI) | GP3 | GP19 (25) |
| INT | GP28 | GP20 (26) |
| VCC | 3V3 | 3V3(OUT) (36) |
| GND | GND | GND (38) |

Requisitos de hardware que no se ven en el código:

- El **TJA1050 necesita 5 V**. En el módulo hay que separar su alimentación de la del MCP2515, que trabaja a 3.3 V.
- Conecta **CAN H con CAN H**, **CAN L con CAN L** y une las **tierras** de las dos placas.
- Pon el jumper **J1** (terminación de 120 Ω) al menos en un módulo. Sin él el bus no regresa al estado recesivo y los mensajes nunca salen.
