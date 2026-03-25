# ARM-Challenge-RoboIME-2026

🛠️ Entendendo o coder.extrinsic no MATLAB
Ao utilizar o MATLAB Coder ou um bloco MATLAB Function no Simulink para gerar código C/C++, você pode esbarrar em funções do MATLAB que não são suportadas para compilação (como funções de gráficos, scripts de visualização ou bibliotecas externas complexas).

É aqui que entra o coder.extrinsic.

O que essa linha faz?
Matlab
coder.extrinsic('meu_controlador_externo');
Essa linha funciona como um "passe livre". Ela avisa ao compilador do MATLAB: "Não tente traduzir a função meu_controlador_externo para C/C++. Em vez disso, quando o código passar por aqui, chame o motor do próprio MATLAB para executá-la."
