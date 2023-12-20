Controlador de Lemniscata ROS

Este pacote ROS contém código para controlar um robô para seguir uma curva Lemniscata usando controladores de Linearização de Feedback.
Visão geral

OBS: Caso o robo fique travado no meio do campo, favor modificar algum loop rate do sistema. Exemplo: modifique o TS do trajectory generator para 0.07. 

O código consiste em três nós:

     stageros: inicia o simulador de palco com um arquivo mundial especificado.
     trajectória_generator_node: Lida com a geração da trajetória desejada.
     move_example_node: Controla o robô para seguir a trajetória da Lemniscata.

Iniciar arquivo

O arquivo de inicialização configura parâmetros e inicia nós:

xml

<lançamento>
   <!-- Parâmetros para a curva de Lemniscata -->
   <param name="~a" type="double" value="4.0" />
   <param name="~kp" type="double" value="5.0" />
   <param name="~kp1" type="double" value="1.00" />
   <param name="~kp2" type="double" value="1.00" />
   <param name="~d" type="double" value="0.1" />

   <!-- Nós -->
   <node pkg="stage_ros" type="stageros" name="stageros" args="$(find move_example)/ecai21_1.world" />
   <node pkg="move_example" type="trajectory_generator_node" name="trajectory_generator_node" output="screen"/>
   <node pkg="move_example" type="move_example_node" name="move_example_node" output="screen"/>
</launch>

Nó Controlador Lemniscata

O nó principal, move_example_node, implementa um controlador de Linearização de Feedback para seguir a curva Lemniscate. Os elementos principais incluem:

     Assinantes:
         odom: Recebe dados de odometria do robô.
         desejado_twist: Obtém a posição e orientação desejada para o robô.

     Editores:
         cmd_vel: Publica comandos de velocidade linear e angular para controlar o robô.

     Parâmetros:
         a: Meia largura da curva da Lemniscata.
         kp, kp1, kp2: Parâmetros do controlador de Linearização de Feedback.

         d: Parâmetro constante para linearização da realimentação.

     Função principal:
         Inicializa o nó ROS, recupera parâmetros e configura assinantes e editores.
         Implementa um loop que calcula a posição e orientação desejadas na curva Lemniscata.
         Calcula comandos de controle usando controladores de Linearização de Feedback e linearização de feedback.
         Publica comandos de velocidade para controlar o robô.

Uso

     Lançamento:
         Execute o arquivo de inicialização: roslaunch <package_name> <launch_file_name>
     Ajuste de parâmetro:
         Modifique os parâmetros no arquivo de inicialização ou por meio da linha de comando.
     Visualização:
         Visualize o movimento do robô no mundo especificado do simulador de palco.
     Ao controle:
         O nó calcula comandos para fazer o robô seguir a curva Lemniscata com base nas posições desejadas e nos dados de odometria atuais.

Observação

Certifique-se de que os pacotes necessários estejam instalados corretamente e que o arquivo de inicialização aponte para caminhos e parâmetros de arquivo válidos.
Notas

     Certifique-se de que os caminhos corretos sejam fornecidos no arquivo de inicialização para o simulador de palco e geração de trajetória.
     Modifique os parâmetros de ajuste do de Linearização de Feedback ou características de trajetória conforme necessário.

Use apt-get to install the Eigen3 library:
sudo apt update
sudo apt-get install libeigen3-dev

