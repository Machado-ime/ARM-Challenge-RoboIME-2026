function q = trajetoria_quintica_fcn(t, q0, qf, Tf)
%TRAJETORIA_QUINTICA_FCN  Perfil quíntico para uso em bloco MATLAB Function do Simulink.
%
%  Calcula a configuração de juntas q(t) em um instante t qualquer,
%  usando o polinômio quíntico com velocidade e aceleração nulas nas bordas.
%
%  ── COMO USAR NO SIMULINK ───────────────────────────────────────────────
%
%  1. Insira um bloco "MATLAB Function" no seu diagrama.
%
%  2. Cole este código dentro do bloco (ou coloque o arquivo .m no path do
%     MATLAB e chame-o de dentro do bloco com:
%
%         function q = step(t, q0, qf, Tf)
%             q = trajetoria_quintica_fcn(t, q0, qf, Tf);
%         end
%
%  3. Entradas do bloco:
%       t   — tempo atual da simulação (escalar, vem do bloco "Clock")
%       q0  — configuração inicial  [6×1] rad  (constante, de "Constant")
%       qf  — configuração final    [6×1] rad  (constante, de "Constant")
%       Tf  — duração da trajetória [escalar]  s  (constante, de "Constant")
%
%  4. Saída do bloco:
%       q   — vetor de ângulos [6×1] rad no instante t
%
%  5. Conecte a saída "q" ao bloco "Joint Position Reference" ou equivalente
%     da simulação do UR5e.
%
%  ── NOTAS ───────────────────────────────────────────────────────────────
%
%  • Para t < 0  : mantém q0 (proteção contra atrasos na inicialização).
%  • Para t >= Tf: mantém qf (robô permanece na posição final).
%  • O perfil quíntico garante q(0)=q0, q(Tf)=qf,
%    dq(0)=dq(Tf)=0, ddq(0)=ddq(Tf)=0.
%
%  ── ASSINATURA COMPATÍVEL COM MATLAB FUNCTION BLOCK ─────────────────────
%  Todos os argumentos devem ter tipo/tamanho fixo declarado no bloco.
%  Declare q0 e qf como "double, 6×1, variable size: off".

% ── Garante vetores coluna ───────────────────────────────────────────────
q0 = q0(:);
qf = qf(:);

% ── Satura t no intervalo [0, Tf] ────────────────────────────────────────
t_sat = max(0, min(t, Tf));

% ── Parâmetro normalizado ─────────────────────────────────────────────────
tau = t_sat / Tf;

% ── Polinômio quíntico (escalar de posição) ───────────────────────────────
%   s(tau) = 10τ³ − 15τ⁴ + 6τ⁵
s = 10*tau^3 - 15*tau^4 + 6*tau^5;

% ── Configuração interpolada ──────────────────────────────────────────────
q = q0 + s * (qf - q0);

end
