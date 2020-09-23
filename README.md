# Interface para controle semáforico simulado

Este projeto contém uma interface para realização de simulações que avaliam diferentes técnicas de controle semafórico adaptativo. Para tal, cria um modelo de controlador semafórico que é executado em threads independentes e realiza a comunicação entre os diversos serviços por meio de um broker de mensagens.

## Descrição do projeto

O projeto é composto, de maneira geral, das entidades:
- Controladores semafóricos
- Central de controle semafórico (agregadora de dados)
- Otimizador (executado sob demanda pela central)
- Interface com a simulação

Cada uma destas será descrita em maiores detalhes a seguir. 

### Controladores

Os controladores semafóricos tem a função básica de acompanhar o passar do tempo e enviar comandos que sinalizem para uma mudança nos estados dos semáforos sempre que necessário. Além disso, eles devem estar atentos para possíveis comandos da central semafórica que indiquem mudanças nos planos que executam.

Os controladores semafóricos também avisam em determinados momentos, assim que finalizam a execução de um ciclo semafórico. Desta forma, a Central de controle sabe em que momentos deve realizar uma nova chamada ao otimizador.

### Central de controle semafórico

A central de controle recebe atualizações da interface com a simulação a cada iteração, atualizando os estados reais de cada via e cada faixa, seguindo parâmetros configuráveis. Além de guardar os dados reais de cada elemento da simulação, a central também é responsável por executar o otimizador sempre que algum controlador finaliza um ciclo.

### Otimizador

Este elemento é responsável por decidir quais serão os tempos executados pelos controladores semafóricos. Sempre que um controlador semafórico finaliza um ciclo de execução, este elemento é chamado e recalcula todos parâmetros da simulação novamente.

### Interface com a simulação

A interface é responsável por gerenciar a passagem do tempo (relógio), ordenar que a simulação dê passos, adquirir os dados do estado das vias e faixas e também alterar os estados dos semafóros sempre que for ordenada. A interface também realiza a leitura de detectores, se estes forem usados.  

## Instalação e uso

Como este projeto utiliza o SUMO e, portanto, o módulo TraCI (Traffic Control Interface), é dependente da instalação local. Não se recomenda que seja executado em ambientes virtuais ou conteineres.  
Da parte do SUMO, é necessário que a variável `$SUMO_HOME` esteja definida corretamente e que o diretório `tools`, dentro do local de instalação do SUMO, esteja no `PATH` do sistema.  

Mais informações sobre o SUMO e o TraCI:  
[SUMO](https://sumo.dlr.de/docs/index.html)  
[TraCI](https://sumo.dlr.de/daily/pydoc/traci.html)  

Para utilizar este projeto, é necessário ter uma instalação local do `RabbitMQ`. Este é um broker de mensagens simples e fácil de utilizar, ao mesmo tempo que fornece uma flexibilidade razoável nas mensagens a serem trocadas.  

Mais informações sobre o RabbitMQ:  
[RabbitMQ](https://www.rabbitmq.com/)  

A instalação é feita com:  
```
python -m pip install -r requirements.txt
```

Para o uso, pode-se utilizar os arquivos de configuração padrão que já estão incluídos no projeto, no diretório `/config`. Para a visualização final dos dados, pode-se utilizar o processamento de saída facilitado pelo script `utils/results_report.py`.

Para a execução de uma simulação, basta fazer:
```
python main.py
```