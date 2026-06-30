/******************************************************************************
 * PROJETO: Torre de Monitoramento LiDAR - Aegis Rover / Rocket Tank
 * SENSOR:  LiDAR LDROBOT LD14P
 * AUTOR:   CFSJ TECH - Engenharia de Computação & Sistemas
 *
 * DESCRIÇÃO:
 * Modelo 3D parametrizado com base no Memorial Descritivo. Inclui flange
 * alargada com rebaixos (counterbores) para parafusos M3, coluna central
 * estruturada, alojamento assimétrico para o motor/placa adaptadora do
 * LD14P e dutos ocos para gerenciamento limpo do cabo flat até o chassi.
 *
 * DIRETRIZES DE FATIAMENTO (SLICER):
 * - Material: PETG, ABS ou ASA (NÃO usar PLA).
 * - Perímetros: Mínimo 4 paredes (garante resistência ao torque nos furos).
 * - Preenchimento (Infill): 30% a 40% Giroide (Gyroid) para absorção de impacto.
 * - Suportes: Tree Supports (apenas para o berço lateral da placa).
 *****************************************************************************/

/* [1. Interface Inferior: Base de Fixação (Chassi)] */
// Distância entre furos no eixo X (Cota modular do chassi)
base_pitch_x = 63.0;
// Distância entre furos no eixo Y (Cota modular do chassi)
base_pitch_y = 100.0;
// Espessura maciça da flange
base_espessura = 4.5; // [4.0 : 0.1 : 5.0]
// Diâmetro passante do parafuso M3
base_furo_passante_d = 3.4;
// Diâmetro do rebaixo para a cabeça do parafuso (Counterbore)
base_rebaixo_d = 6.2;
// Profundidade do rebaixo
base_rebaixo_prof = 3.0;
// Raio de arredondamento das pontas da flange
base_raio_canto = 6.0;

/* [2. Interface Superior: Alojamento LD14P] */
// Avanço esquerdo do centro (Orelha esquerda)
lidar_raio_esq = 35.3;
// Avanço direito do centro (Placa adaptadora)
lidar_raio_dir = 36.0;
// Vão livre central para o motor "flutuar" (Mínimo 35mm)
lidar_motor_clearance_d = 36.0;
// Espessura da base sólida superior
lidar_base_espessura = 3.5;
// Diâmetro do furo M2 (orelhas de fixação)
lidar_furo_m2_d = 2.2;
// Largura da plataforma superior (eixo Y)
lidar_largura_y = 38.0;

/* [3. Corpo Central: Coluna de Elevação] */
// Altura da torre (Da chapa do chassi até a base do LiDAR)
torre_altura = 50.0; // [40.0 : 1.0 : 60.0]
// Diâmetro externo da coluna na base
torre_d_base = 35.0;
// Diâmetro interno do duto central (Furo Verde - Mínimo 10mm)
duto_central_d = 12.0;

/* [Resolução] */
$fn = 100; // Alta resolução para curvas suaves

// ============================================================================
// MÓDULO PRINCIPAL (OPERAÇÕES BOOLEANAS)
// ============================================================================

difference() {
    // GEOMETRIA POSITIVA (Estrutura Sólida)
    union() {
        // 1. Flange da Base
        construir_base();

        // 3. Coluna Central (Transição cônica suave)
        construir_coluna();

        // 2. Plataforma Superior do LiDAR
        construir_plataforma_superior();
    }

    // GEOMETRIA NEGATIVA (Cortes, Furos e Dutos)
    union() {
        // 1. Furações da Base (com Rebaixo/Counterbore)
        furos_fixacao_base();

        // 2. Vão livre do motor do LiDAR
        clearance_motor_lidar();

        // 2. Berço/Rasgo para a Placa Adaptadora e Conector JST
        berco_placa_adaptadora();

        // Furos de fixação do LiDAR nas extremidades
        furos_fixacao_lidar();

        // 3. Duto Central (Gravidade/Fiação)
        duto_fiacao_central();

        // Duto em ângulo conectando o berço da placa ao tubo central
        duto_fiacao_transicao();

        // Chanfro inferior anti-pé-de-elefante
        anti_elephant_foot();
    }
}

// ============================================================================
// MÓDULOS DE CONSTRUÇÃO POSITIVA
// ============================================================================

module construir_base() {
    // Flange retangular com cantos arredondados cobrindo os furos
    hull() {
        for (x = [-1, 1]) {
            for (y = [-1, 1]) {
                translate([x * base_pitch_x/2, y * base_pitch_y/2, 0])
                cylinder(r=base_raio_canto, h=base_espessura);
            }
        }
    }
}

module construir_coluna() {
    // Pilar central que une a base à plataforma superior
    hull() {
        // Círculo na base (fundido com a flange)
        cylinder(d=torre_d_base, h=base_espessura);
        // Círculo no topo (abaixo da plataforma do LiDAR)
        translate([0, 0, torre_altura - lidar_base_espessura])
        cylinder(d=lidar_largura_y, h=lidar_base_espessura);
    }
}

module construir_plataforma_superior() {
    // Geometria assimétrica exata (35.3mm para a esquerda, 36.0mm para a direita)
    translate([0, 0, torre_altura - lidar_base_espessura])
    hull() {
        // Extremidade Esquerda (-X)
        translate([-lidar_raio_esq + 3, 0, 0])
        cylinder(r=3, h=lidar_base_espessura);

        // Extremidade Direita (+X)
        translate([lidar_raio_dir - 3, 0, 0])
        cylinder(r=3, h=lidar_base_espessura);

        // Corpo central encorpado
        cylinder(d=lidar_largura_y, h=lidar_base_espessura);
    }
}

// ============================================================================
// MÓDULOS DE CORTE NEGATIVO
// ============================================================================

module furos_fixacao_base() {
    // Furos M3 com rebaixos profundos para embutir a cabeça do parafuso
    for (x = [-1, 1]) {
        for (y = [-1, 1]) {
            translate([x * base_pitch_x/2, y * base_pitch_y/2, -1]) {
                // Furo passante (3.2 a 3.4mm)
                cylinder(d=base_furo_passante_d, h=base_espessura + 2);

                // Rebaixo cilíndrico superior (Counterbore)
                translate([0, 0, base_espessura - base_rebaixo_prof + 1])
                cylinder(d=base_rebaixo_d, h=base_rebaixo_prof + 1);
            }
        }
    }
}

module clearance_motor_lidar() {
    // Vão livre central passante no topo para a polia/motor não encostar (Mín 35mm)
    translate([0, 0, torre_altura - 15]) // Desce 15mm adentro da coluna
    cylinder(d=lidar_motor_clearance_d, h=20);
}

module berco_placa_adaptadora() {
    // Rasgo no lado direito (+X) para acomodar a placa/conector JST
    // Feito cortando a plataforma superior a partir da borda do clearance do motor
    translate([lidar_motor_clearance_d/2 - 2, -10, torre_altura - lidar_base_espessura - 2])
    cube([lidar_raio_dir - (lidar_motor_clearance_d/2) + 5, 20, lidar_base_espessura + 5]);
}

module duto_fiacao_central() {
    // Duto vertical central para passagem livre do cabo (Furo Verde)
    translate([0, 0, -1])
    cylinder(d=duto_central_d, h=torre_altura + 2);
}

module duto_fiacao_transicao() {
    // Cria um duto interno suave (45 graus) ligando o berço lateral ao duto vertical
    // Evita ângulos de 90 graus para não amassar o cabo flat
    hull() {
        translate([lidar_motor_clearance_d/2, 0, torre_altura - 3])
        sphere(d=duto_central_d * 0.8);

        translate([0, 0, torre_altura - 20])
        sphere(d=duto_central_d);
    }
}

module furos_fixacao_lidar() {
    // Furações de 2.0 a 2.2mm nas extremidades sólidas para fixar o LD14P
    // Esquerda
    translate([-lidar_raio_esq + 3, 0, torre_altura - lidar_base_espessura - 1])
    cylinder(d=lidar_furo_m2_d, h=lidar_base_espessura + 2);

    // Direita (Opcional, pois a placa fica neste lado, mas adicionado para simetria de apoio)
    translate([lidar_raio_dir - 3, 0, torre_altura - lidar_base_espessura - 1])
    cylinder(d=lidar_furo_m2_d, h=lidar_base_espessura + 2);
}

module anti_elephant_foot() {
    // Pequeno chanfro no perímetro inferior (0.4mm) para compensar
    // o esmagamento da primeira camada na impressão 3D
    difference() {
        translate([0,0,-0.1])
        cube([base_pitch_x + 30, base_pitch_y + 30, 0.6], center=true);

        hull() {
            for (x = [-1, 1]) {
                for (y = [-1, 1]) {
                    translate([x * base_pitch_x/2, y * base_pitch_y/2, -0.1])
                    cylinder(r=base_raio_canto - 0.4, h=1);
                }
            }
        }
    }
}
