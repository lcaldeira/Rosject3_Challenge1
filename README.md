## Enunciado

Esse projeto consiste no desafio **"Siga o Mestre"**, em que um *turtlebot* é considerado o mestre e será controlado pelo *teleop*. Os outros robôs devem seguí-lo pelo mapa, respeitandoa distância mínima.

Membros:
- Lucas Caldeira de Oliveira
- Lucas Volkmer Hendges

## Configurações do Ambiente

A importação dos submódulos e configurações iniciais é dada abaixo.

> `$ echo "
cd catkin_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger" >> ~user/.bashrc`
>
> `$ cd catkin_ws/src`
>
> `$ git init`
>
> `$ git submodule add --branch noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git`
>
> `$ git submodule add --branch noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
>
> `$ git submodule add --branch noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

## Criação do Pacote

Será criado um pacote para o projeto, que conterá o *script* responsável pela tópico de odometria.

> `$ catkin_create_pkg turtle_fleet std_msgs rospy`
>
> `$ cd turtle_fleet`
>
> `$ mkdir launch script`

Também serão importados alguns arquivos do projeto **Rosject 2 - Turtlebot**, que servirão de base para o restante.

> `$ curl https://raw.githubusercontent.com/lcaldeira/Rosject2_Turtlebot/main/turtleparty/launch/two_tb3.launch > launch/two_tb3.launch`
>
> `$ curl https://raw.githubusercontent.com/lcaldeira/Rosject2_Turtlebot/main/turtleparty/script/repeat_teleop.py > script/convoy.py`
>
> `$ sudo chmod +x script/convoy.py`

O arquivo `convoy.py` será alterado para que a interceptação dos comandos de *teleop* sejam direcionados apenas a ele.

## Rodando o Projeto

Para rodar o projeto, abra 1 terminal (com as variáveis de ambiente configuradas) e digite:

> `$ catkin_make`
>
> `$ roslaunch turtle_fleet two_tb3.launch`
>
> `Ctrl+Z`
>
> `$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

Abra o *gazebo* e observe a dança dos robôs :D

## Salvando no GitHub

Um pequeno lembrete passo-a-passo de como usar o git.

Para salvar o projeto, incluindo os submódulos, execute os comandos abaixo na pasta `catkin_ws/src`.

> `$ git add turtle_fleet CMakeList.txt`
>
> `$ git remote add origin https://<token>@github.com/lcaldeira/Rosject3_Challenge1.git`
>
> `$ git pull origin main`
>
> `$ git checkout main`
>
> `$ git config user.name "your_name"`
>
> `$ git config user.email "your@email.com"`
>
> `$ git commit -m "first upload to github"`
>
> `$ git push origin main`
