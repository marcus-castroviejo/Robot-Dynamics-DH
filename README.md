# Robot-Dynamics-DH
Kynematics, Dynamics and Control of a robot using the Denavit Hartenberg parameters.

### Código Principal: 
- Arquivo: **dinamica_2_codigo.ipynb**
- Primeiramente é criada a classe Robot que possibilita o cálculo da cinemática e dinâmica do robô, com implementação com foco na utilização de variáveis simbólicas, possibilitando fácil alteração de qualquer parâmetro ou configuração do robô.
- Em seguida, o robô é definido. A classe já apresenta valores defaults para as variáveis simbólicas, mas algumas alterações são desejadas (por exemplo, valores que são nulos facilitam e agilizam o código). A Tabela com parâmetros de DH deve ser fornecida para instanciação da classe.
- Em seguida, podemos visualizar os resultados fornecidos pela classe.
- Então parte-se para a substituição dos parâmetros simbólicos com os valores reais desejados. A mudança dos parâmtros e variáveis é realizada de forma iterativa, onde fica simples visualizar o que é feito a cada passo.
- A trajetória desejada é implementada e calculada e os valores máximos das matrizes de Inércia e de Coriolis são calculados (vão ser usados para cálculo dos ganhos do controlador PID).
- Cálculo dos ganhos do Controlador PID e implementação. Settling time é escolhido como $t_s = 0.3s$, e define-se um overshoot máximo de 10% ($\zeta = 0.59$). Então calcula-se $\omega_n = \frac{3}{\zeta t_s}$ e um pólo distante $p = 5\zeta\omega_n$. Pode-se escolher entre PID clássico ou com Torque Calculado, mas o foco para este trabalho é o PID clássico.
- Geração dos Gráficos. Basta selecionar qual variável será plotada ($q(t), \dot q(t), \ddot q(t), e(t), \tau(t)$). No caso da visualização individual, selecionar qual junta será exibida. À cada junta é atribuída uma cor específica, que se mantem em todos os gráficos para manter conformidade.

### Comparação com a Biblioteca de Peter Corke:
- Arquivos: **teste_peter_corke.ipynb**, **teste_peter_corke.py**
- Arquivos criados para comparar os resultados obtidos pela implementação própria com a implementação da Biblioteca do Peter Corke. Além disso, com o arquivo .py, pudemos gerar uma simulação que segue a trajetória desejada (**simulacao.mp4**)

### Gráficos
Os gráficos foram salvos na pasta **Graficos**.