UFPE - CTG
ALunos: Sanclay Augusto e Emerson Guilherme
Programação Robótica
Trabalho 2

- Ultrassom.py (nó)
PUBLICA PARA O NÓ ecu.py
Pega o valor da distância e manda para o nó ecu.py a informação da distâncoa.

- visao.py (nó)
PUBLICA PARA ECU
INSCRITO DA ECU
Recebe o comando para que ele realize a análise da visão, movimenta o servo para esquerda, tira uma foto, analisa 
o servo vai para direita, tira a foto e analisa. O resultado da análise é inviado para a ecu.py

- ecu.py (nó)
PUBLICA PARA RODAS
PUBLICA PARA VISAO
INSCRITO DA VISAO
INSCRITO DO ULTRASSOM
Contem a lógica da movimentação do robô, analisa os dados recebidos do umtrassom e da visao, manda informações para 
rodas, que realiza o movimento e manda informações para a visão atuar

- Rodas.py (nó)
INSCRITO DE ECU
Recebe os comandos para executar os movimentos, frente, esquerda 90º, direita 90º e pare.