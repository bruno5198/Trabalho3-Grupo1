# Team Hunt (PSR - Trabalho3 Grupo1)
Repositório para a partilha de conteúdos relativos ao trabalho prático número 3, realizado no âmbito da Unidade Curricular de Programação de Sistemas Robóticos (PSR) pelos alunos da Universidade de Aveiro:
###### [André Vasconcelos (88983)](https://github.com/andredvasconcelos);
###### [Bruno Mendes (83583)](https://github.com/bruno5198);
###### [Ivo Bastos (93194)](https://github.com/IvoBastos);
###### [Martín Rivadeneira (104548)](https://github.com/MartinRivadeneira).

# Indice
 - [Instalação](#instalação);
 - [Launch files de lançamento do robô](#launch-files-de-lançamento-do-robo);
 - [Simulação em Gazebo](#simulação-em-gazebo);
 - [Vizualização do robô em RViz](#vizualização-do-robô-em-rviz);
 - [Manual driving race (Teleop)](#manual-driving-race-teleop);
 - [Seguimento de um Goal (RViz)](#seguimento-de-um-goal-rviz);
 - [Interface do nó driver](#interface-do-no-driver);
 - [Perceção da cena](#perceção-da-cena);
 - [Modo de fuga](#modo-de-fuga);
 - [Modo de perseguição](#modo-de-perseguição);
 - [Evitar obstáculos](#evitar-obstáculos);
 - [Mapeamento do cenário](#mapeamento-do-cenário);
 - [Variabilidade de drivers](#variabilidade-de-drivers);
 - [Árbitro](#arbitro);
 - [Extras](#extras);
 - [Demonstração do jogo](#demonstração-do-jogo).

# Instalação
Como ponto de partida deve certificar-se de que possui o [ROS](http://wiki.ros.org/ROS/Introduction) instalado. Caso necessário, pode consultar um guia de instalação [aqui](http://wiki.ros.org/ROS/Installation). Salientar que, todo o trabalho presente neste repositório fora desenvolvido sobre o sistema [ROS Noetic](http://wiki.ros.org/noetic).

Depois de instalado o ROS, pode efetuar o [dowload do presente repositório](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository). O mesmo contém duas pastas, com três pacotes ROS principais no seu interior, que permitem o controlo de um ou mais robôs com base em duas abordagens/dois códigos distintos.

O pacote [p_bmendes_bringup](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes/p_bmendes_bringup) (ou [p_mrivadeneira_bringup](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_bringup)) contém um ficheiro onde constam diversos parâmetros necessários ao funcionamento do jogo TeamHunt, um conjunto de ficheiros de configuração do RViz e, por fim, um conjunto de ficheiros do tipo launch files através dos quais é, por exemplo, lançado o ambente de simulação do Gazebo e definido como é efetuado o spawn de cada robô.

O pacote [p_bmendes_description](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes/p_bmendes_description) (ou [p_mrivadeneira_description](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_description)) contém um conjunto de ficheiros onde é definida a geometria do robô bem como os sensores que o mesmo possui, como é exemplo o sensor de contacto necessário ao funcionamento do jogo TeamHunt. Nos ficheiros em causa são ainda definidas um conjunto de propriedades essenciais, nomeadamente as cores que que poderão ser utilizadas no jogo e que representam, por exemplo, cada uma das três equipas existentes (equipa azul, verde e vermelha).

Por fim, o pacote [p_bmendes_player](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes/p_bmendes_player) (ou [p_mrivadeneira_player](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_player)) contém o algoritmo desenvolvido e que permite o controlo de um ou mais robôs, tendo sido, portanto, desenvolvidos dois algoritmos distintos.

# Launch files de lançamento do robo
Para o lançamento não só do robô mas de tudo aquilo que é necessário para a realização do jogo TeamHunt (como, por exemplo, a arena onde decorre o jogo ou o árbitro que monitoriza o mesmo) foi criado o ficheiro [game_bringup.launch](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_bmendes/p_bmendes_bringup/launch/game_bringup.launch) (ou [game_bringup.launch](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_bringup/launch/game_bringup.launch).

Através do ficheiro em causa é possível fazer o spawn de vários robôs, colocando-os em equipas de acordo com o seu nome, através dos parâmetros predefinidos no dicheiro [game.yaml](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_bmendes/p_bmendes_bringup/params/game.yaml) (ou [game.yaml](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_bringup/params/game.yaml) na versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira)).

Parâmetros como o nome do robô, a sua cor ou a [arena](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/TeamHunt/th_description) na qual se pretende que decorra o jogo podem ser alterados consoante o pretendido. Para isso basta executar o seguinte comando (para a versão [p_bmendes](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes)) num terminal, substituindo os campos entre parêntises retos de acordo com o pretendido:

    roslaunch p_bmendes_bringup game_bringup.launch player_name:=[nome] player_color:=[cor] x_pos:=[X axis position] y_pos:=[Y axis position] z_pos:=[Z axis position]

Ou o seguinte comando (para a versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira)):

    roslaunch p_mrivadeneira_bringup game_bringup.launch reds:=[red players number] blues:=[blue players number] greens:=[green players number]

# Simulação em Gazebo
Para a criação de um ambiente simulado, em [Gazebo](http://gazebosim.org/), abra um terminal e corra o seguinte comando (para a versão [p_bmendes](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes)):

    roslaunch p_bmendes_bringup gazebo.launch

Ou o seguinte comando (para a versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira)):

    roslaunch p_mrivadeneira_bringup gazebo.launch arena:=[arena]

Este comando fará despoletar o simulador Gazebo, já com o mundo/arena referente ao jogo TeamHunt.

![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/Gazebo%20simulator.png)


# Vizualização do robô em RViz
Para a visualização do robô em [RViz](http://wiki.ros.org/rviz), software de vizualização 3D, e respetios dados associados ao robô, como as imagens captadas pelas câmaras que constituem o robô, a informação referente ao Laserscan captada pelo robô, entre outros, deve correr o seguinte comando, para a versão [p_bmendes](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes), num terninal:

    roslaunch p_bmendes_bringup visualize.launch player_name:=[nome]

Ou o seguinte comando (para a versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira)):

    roslaunch p_mrivadeneira_bringup visualize.launch player_name:=[nome]

![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/Robot%20visualization%20RViz.png)

# Manual driving race (Teleop)
O controlo manual do robô pode ser efetuado de duas formas distintas, recorrendo ao [controlador do ROS](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_bringup/launch/myteleop.launch) (com ligeiras modificações), ou recorrendo a um telemóvel.

Para o ontrolo do robô através do controlador do ROS deve executar o seguinte comando num terminal:

    roslaunch p_mrivadeneira_bringup myteleop.launch player_name:=[nome]
    
Para tornar a condução manual de um robô mais interessante, pode controlar o mesmo com o recurso a um telemóvel que desempenha a função de um joystick. Para isso é necessário a instalação prévia, no seu smartphone, da aplicação [ROS Control](https://play.google.com/store/apps/details?id=com.robotca.ControlApp).

Depois de instalada a referida aplicação, deve adicionar um novo robô (como mostra a figura da esquerda), parametrizando-o em função do nome do robô que pretende controlar, do IP do computador onde está a correr a simulação em Gazebo e do nome dos tópicos que pretende controlar (como mostra a figura da direita). Posto isto o robô pode então ser controlado, por exemplo, através da função de joystick que a apicação disponibiliza.

Adição de um novo robô     |  Parametrização do robô
:-------------------------:|:-------------------------:
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/ROS_Application_Add_Robot.jpg)  |  ![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/ROS_Application-Parameterize_Robot.jpg)

###### Clique na imagem para ver um pequeno video demonstrativo da condução manual de um robô.
!!!!!!!!! Colocar aqui um vídeo do controlo manual do robô !!!!!!!!!!

**Nota:** Para que seja possível o controlo do robô como descrito anteriormente, é neessário que o computador onde está a correr a simulação em Gazebo e o telemóvel através do qual se pretende controlar o robô estejam na mesma rede.

# Seguimento de um Goal (RViz)
Para o seguimento de um Goal, ao qual se deu precedência sobre os restantes comportamentos do robô (o comportamento de caça e de fuga, por exemplo), fora inicialmente utilizada uma abordagem simplista, visível na versão [p_bmendes](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes), onde após a definição de um Goal, no RViz, o robô se desloca em linha reta até alcançar esse mesmo Goal, não considerando eventuais obstáculos pelo caminho. No sentido de tornar mais eficiente esta componente, optou-se por uma segunda abordagem, visível na versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira), em que se recorrera ao [ROS Navigation](http://wiki.ros.org/navigation), permitindo assim que o robô não só se movimentasse em direção a um Goal previamente definido, mas fazendo-o evitando os obstáculos que surgem no seu caminho.

![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/Navigation.png)

Notar que, para a componente de navegação são necessários os seguintes pacotes:

 - [ROS navigation](https://github.com/ros-planning/navigation);
 - [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3).

###### Clique na imagem para ver um vídeo referente a uma aula de esclarecimento de dúvidas onde fora abordada esta questão.
[![Watch the video](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/ROS_Navigation_Aula_D%C3%BAvidas.png)](https://www.youtube.com/watch?v=WWUOUchK3Q4&list=PLQN09mzV5mbLK9OvKQf1ZfXpPlBi461zp&index=41)

# Interface do no driver
Com o intuito de tornar simples a perceção do comportamento do robô em cada instante, foram implementadas algumas "técnicas" como por exemplo, na versão [p_bmendes](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_bmendes), a criação de uma janela através da qual podem ser visualizadas as imagens provenientes das câmaras que o robô possui (uma câmara a apontar para a frente e outra para trás), bem como uma legenda que permite perceber a qual dos robôs corresponde a imagem (bem como qual a sua equipa e adversários), se está a ser detetada alguma presa ou caçador e qual o estado do robô, isto é, se se encontra em modo de fuga, em modo de perseguição ou em modo de procura por uma presa.

Interface p_bmendes     
:-------------------------:
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/CV2%20Window%20p_bmendes.png)

Na versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira), fora também criada uma janela através da qual pode ser visualizada a câmara frontal que o robô possui, janela essa através da qual se consegue ter a perfeita precessão do caminho que o robô terá de percorrer para alcançar um Goal manualmente definido no RViz, como mostra a imagem seguinte.

Interface p_mrivadeneira     
:-------------------------:
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/CV2%20Window%20p_mrivadeneira.png)
Seguimento de um Goal p_mrivadeneira
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/Goal.png)

# Perceção da cena
Relativamente à perceção da cena, cada robô consegue ter uma noção daquilo que o rodeia, tendo sido utilizados para isso diversos sensores, como são exemplo o Lasrscan, as câmeras, entre outros.
A título de exemplo, através da/das câmaras e do processamento de imagem, cada robô consegue detetar a presença de uma presa ou de um caçador e tomar decisões em função da posição dos mesmos. Recorrendo à informação obtida através do Laserscan, cada robô consegue ainda detetar obstáculos, como paredes, perceber qual a posição dos mesmos e evitar uma eventual colisão.

# Modo de fuga
Para o modo de fuga, optou-se por dotar os robôs da capacidade de, assim que detetem um caçador, através do processamento da imagem proveniente da sua câmara frontal, rodarem rápidamente 180 graus para que esse mesmo caçador fique nas suas costas e assim possam andar em frente até que deixem de estar "sob ameaça". Para facilitar a fuga optou-se por recorrer a uma segunda câmara, capaz de visualizar e interpretar o que se passa nas costas do robô. Através desta segunda câmera o robô que está em fuga consegue perceber se ainda está a ser perseguido por um caçador e qual a distância a que o mesmo se encontra. Caso o caçador se encontre a uma distância definida como segura, o modo de fuga é interrompido e o robô inicia o modo de procura por uma presa.

# Modo de perseguição
No modo de perseguição, o robô segue em linha reta em direção à sua presa, previamente detetada através do processamento da imagem proveniente da sua câmara frontal. Caso essa mesma presa se desloque, tentando evitar ser caçada, o robô que se encontra em perseguição compensa a sua direção no sentido de necessitar apenas de se movimentar em linha reta.

# Evitar obstáculos
Para evitar obstáculos, foram seguidas duas abordagens. Numa das abordagens, o robô recorre ao sensor [LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) para ter uma perseção daquilo que o rodeia e, considerando que um obstáculo (tipicamente, uma parece) tem sempre um tamanho superior quando comparado com um robô, conseguir afastar-se de um qualquer obstáculo.

Numa segunda abordagem, como referido anteriormente na secção [Seguimento de um Goal (RViz)](#seguimento-de-um-goal-rviz), recorreu-se ao ROS Navigation, onde o robô consegue ir ao encontro de um qualquer objetivo (quer o mesmo seja manualmente definido no RViz, quer corresponda, por exemplo, a uma presa) evitando eventuais obstáculos (podendo esses mesmos obstáculos tratar-se por exemplo, de um caçador).

# Mapeamento do cenário
Para o mapeamento do cenário recorreu-se ao pacote [gmapping](http://wiki.ros.org/gmapping), através do qual é efetuado um mapeamento da cena à medida que cada robô se vai movimentando, como mostra a figura seguinte.

![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/gmapping.png)

# Variabilidade de drivers
Como fora abordado ao longo das secções anteriores, o presente projeto conta com dois drivers distintos, um dos quais se foca na componente de fuga e perseguição e o outro na aquisição da maior quantidade de informação para uma melhor perceção da cena.

# Arbitro
Para a monitorização do jogo, foi implementado um [árbitro](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/TeamHunt/th_referee/src/th_referee) (código criado pelo professor da Unidade Curricular).
O árbitro pode ser lançado/executado correndo o seguinte comando num terminal:

    rosrun TeamHunt_th_referee th_referee

**Nota:** Na versão [p_mrivadeneira](https://github.com/bruno5198/Trabalho3-Grupo1/tree/main/Trabalho3-Grupo1/p_mrivadeneira) basta executar o [game_bringup.launch](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/p_mrivadeneira/p_mrivadeneira_bringup/launch/game_bringup.launch) que o árbitro já é automaticamente executado.

# Extras
###### Adição de "braços" aos robôs
A adição de "braços" aos robôs tinha como objetivo inicial facilitar o modo de perseguição, sendo que assim que um robô se encontrasse próximo de uma presa "abriria os braços" apanhando/tocando na mesma. Contudo, uma vez que o sensor de contacto se encontra restringido ao corpo do robô, tal abordagem não foi possível. Optou-se assim por dar uma outra utilidade à adição de braços aos robôs, ainda no sentido de facilitar o mode de perseguição. Quando um robô se encontra próximo de uma presa, "abre os braços" numa tentativa de tornar mais difícil a tarefa de fuga da sua presa.

Para que os controladores dos braços dos robôs funcionem é necessário instalar os seguintes pacotes:

 - [Ros control](https://github.com/ros-controls/ros_control);
 - [Ros controllers](https://github.com/ros-controls/ros_controllers).

**Nota:** No pacote "Ros controllers" deve ser apagado o pacote "four_wheel_steering_controller" pois apresenta um erro de compilação.

# Demonstração do jogo
De seguida serão apresentadas algumas imagens, bem como um pequeno vídeo demonstratido do jogo TeamHunt.

Imagens 1     |  Imagem 2
:-------------------------:|:-------------------------:
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/TeamHunt%20Game%20Example%201.png)  |  ![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/TeamHunt%20Game%20Example%202.png)

Resultados do jogo
:-------------------------:
![Real Image](https://github.com/bruno5198/Trabalho3-Grupo1/blob/main/Trabalho3-Grupo1/docs/TeamHunt%20Game%20Results.png)
