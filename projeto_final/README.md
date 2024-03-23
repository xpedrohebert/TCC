| Dispositivos suportados | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# Trabalho Final: Medidor de Energia absorvida baseado no exemplo "HC-SR04 Example based on MCPWM Capture" e nas práticas realizadas em turma

(Ler o arquivo README.md no diretório 'sample project' para mais informações sobre o trabalho.)

Este trabaLho foi desenvolvido com a ideia de medir a energia abservida em um ensaio Charpy Através do angulo final do sistema. Para tal foi usado dois sensores encoder disponíveis no link: https://produto.mercadolivre.com.br/MLB-3699805434-sensor-de-velocidade-fotoeletrico-com-encoder-arduino-pic-_JM#position=44&search_layout=stack&type=item&tracking_id=7f39a904-02c7-4bb5-9778-c331f98c39b8. Os sensores geram pulsos defasados um do outro que sao usados para contar a quantidade de interrupções no sensor óptico e assim calcular o angulo. 

Para esse sistema um pendulo é soltado de um angulo X e deveria ir até um angulo -X porem chega somente em -Y por conta da energia absorvida no ensaio. Os pulsos e bordas são gerados pelo giro do péndulo de forma que o sinal gerado na saída dos sensores será dada da seguinte forma:

Sinal Típico:

```

Sensor 1    +-----+     +-----+     +---+
            |     |     |     |     |   |
            |     |     |     |     |   |
         ---+     +-----+     +-----+   +----

Sensor 2       +-----+     +-----+
               |     |     |     |
               |     |     |     |
         ------+     +-----+     +-----------

 +------------------------------------------->
                    Timeline
```

Através da ocorrencia de duas interrupções em um sensor é identificado a inversão do sistema e a contágem para. Com o módulo de captura pegamos o tempo do ultimo pusso que usamos para refinar a medida de angulo. O sinal do sensor deve ser ligado a um sensor amplificador de ganho 2 já que sua saída é de 1.5V e precisa ser aplificado para 3V.

Para liberar a contágem e para habilitar outra medição/ensaio um botão deve ser precionado para que a contágem zere e para que o valor a contágem seja habilitada.

## Como usar o medidor

### Requisição de Hardware 

* Uma placa de desenvolvimento ESP que possua o periférico MCPWM
* Sensor encoder https://produto.mercadolivre.com.br/MLB-3699805434-sensor-de-velocidade-fotoeletrico-com-encoder-arduino-pic-_JM#position=44&search_layout=stack&type=item&tracking_id=7f39a904-02c7-4bb5-9778-c331f98c39b8

Conexão do sistema:

```
           Sensores            Adequação de sinal                           ESP-32
        +-------------+          +------------+           +----------------------------------------+
+-------+             |          |            |           |                                        |
|       |     VCC     +----------+     5V     +-----------+                   5V                   |
+-------+             |          |            |           |                                        |
        + Sensor 1 S1 +--=====>--+ AmpOp Av 2 +--======>--+ GPIO26 (Resistor de pull down Externo) |
        |             |          |            |           |                                        |
        + Sensor 2 S1 +--=====>--+ AmpOp Av 2 +--======>--+ GPIO27 (Resistor de pull down Externo) |
+-------|             |          |            |           |                                        |
|       |     GND     +----------+     GND    +-----------+                  GND                   |
+-------|             |          |            |           |                                        |
        +-------------+          +------------+     +-----+ GPIO25 (Resistor de pull down Externo) |
                                                    |     |                                        |
+-------+ Push button +--=====================>-----+     +----------------------------------------+

```


### Build and Flash

Dê um build e flash o programa no ESP32, então visualize os resultados na saída do terminal serial

(Para sair do terminal serial, apertar "Ctrl + ]".)

## Teste:
 1. Conectar saída 1 do sensor 1 ao amplificador de ganho 2 e a saída do amplificador à porta 26 e conectar saída 1 do sensor 2 à outro amplificador de ganho 2 e a saída do mesmo à porta 27.
 2. Utilizar um push buton e pegar a saída conectada ao GND e ligar a porta 25
 3. Girar o disco detector em um sentido e depois no outro para verificar a saída do sistema

## Problemas dúvidas e sujestões

Para quaisquer dificuldade técnicas abra um [issue](https://github.com/xpedrohebert/sistemas-embarcados/issues) no GitHub. Responderemos assim que possível.