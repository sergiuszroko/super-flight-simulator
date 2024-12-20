use bevy::{
    asset::LoadState,
    core_pipeline::Skybox,
    prelude::*,
    render::render_resource::{TextureViewDescriptor, TextureViewDimension},
};

const G: f32 = 9.81;
const RO: f32 = 1.0;

const M: f32 = 1000.0;
const S: f32 = 18.0;
const L: f32 = 8.0;
const C_MEAN: f32 = 1.5;
const I_X: f32 = 1285.0;
const I_Y: f32 = 1824.0;
const I_Z: f32 = 2666.0;
const I_XZ: f32 = 0.0;

const C_X_DM: f32 = 0.02;
const C_Y_BETA: f32 = -0.05;
const C_Z_0: f32 = 0.2;
const C_Z_ALFA: f32 = 6.0;
const C_Z_DM: f32 = 0.0;
const C_L_BETA: f32 = 0.05;
const C_L_P: f32 = -0.05;
const C_L_R: f32 = 0.05;
const C_L_DELTA_A: f32 = 0.02;
const C_M_ALFA: f32 = -0.5;
const C_M_DELTA_E: f32 = 0.01;
const C_M_Q: f32 = -0.1;
const C_N_BETA: f32 = 0.02;
const C_N_DELTA_R: f32 = 0.01;
const C_N_P: f32 = -0.02;
const C_N_R: f32 = -0.05;
const K_CX: f32 = 0.05;

const THRUST: f32 = 3000.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, (setup_airplane, setup_world))
        .add_systems(
            Update,
            (update_dynamics, update_transform, update_camera, draw_axes).chain(),
        )
        .add_systems(Update, finish_loading_skybox)
        .run();
}

#[derive(Component)]
struct Airplane;
#[derive(Component)]
struct Camera;
#[derive(Component, Debug)]
struct RigidBody {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub phi: f32,
    pub theta: f32,
    pub psi: f32,

    pub u: f32,
    pub v: f32,
    pub w: f32,
    pub p: f32,
    pub q: f32,
    pub r: f32,
}

fn setup_airplane(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(ConicalFrustum {
                    radius_top: 1.0,
                    height: 5.0,
                    radius_bottom: 0.7,
                }),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_z(
                    -std::f32::consts::FRAC_PI_2,
                ))
                .with_translation(Vec3::new(0.0, 1.5, 0.0)),
                ..default()
            },
            Airplane,
            RigidBody {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                phi: 0.0,
                theta: 0.0,
                psi: 0.0,
                u: 0.0,
                v: 0.0,
                w: 0.0,
                p: 0.0,
                q: 0.0,
                r: 0.0,
            },
        ))
        .with_children(|parent| {
            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(3.0, 1.25)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_y(
                    -std::f32::consts::FRAC_PI_2,
                )),
                ..default()
            });
            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(1.8, 0.5)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_y(
                    -std::f32::consts::FRAC_PI_2,
                ))
                .with_translation(Vec3::new(0.0, -2.0, 0.0)),
                ..default()
            });
            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(1.1, 0.5)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_translation(Vec3::new(-0.5, -2.0, 0.0)),
                ..default()
            });

            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(3.0, 1.25)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_y(
                    std::f32::consts::FRAC_PI_2,
                )),
                ..default()
            });
            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(1.8, 0.5)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_y(
                    std::f32::consts::FRAC_PI_2,
                ))
                .with_translation(Vec3::new(0.0, -2.0, 0.0)),
                ..default()
            });
            parent.spawn(PbrBundle {
                mesh: meshes.add(Ellipse::new(1.1, 0.5)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_rotation(Quat::from_rotation_y(std::f32::consts::PI))
                    .with_translation(Vec3::new(-0.5, -2.0, 0.0)),
                ..default()
            });
        });
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    commands.spawn(PbrBundle {
        mesh: meshes.add(Circle::new(10000.0)),
        material: materials.add(Color::srgb(0.0, 1.0, 0.01)),
        transform: Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(20.0, 30.0, 10.0)),
        material: materials.add(Color::srgb(0.0, 0.0, 1.01)),
        transform: Transform::from_translation(Vec3::new(50.0, 15.0, 50.0)),
        ..default()
    });

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    let skybox_handle = asset_server.load("Ryfjallet_cubemap.png");

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-60.0, 60.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        Skybox {
            image: skybox_handle.clone(),
            brightness: 1000.0,
        },
        Camera,
    ));
}

fn finish_loading_skybox(
    mut images: ResMut<Assets<Image>>,
    asset_server: Res<AssetServer>,
    skyboxes: Query<&mut Skybox>,
) {
    let skybox = skyboxes.single();
    if !(asset_server.load_state(&skybox.image) == LoadState::Loaded) {
        return;
    }
    let image = images.get_mut(&skybox.image).unwrap();
    if image.texture_descriptor.array_layer_count() == 1 {
        image.reinterpret_stacked_2d_as_array(image.height() / image.width());
        image.texture_view_descriptor = Some(TextureViewDescriptor {
            dimension: Some(TextureViewDimension::Cube),
            ..default()
        });
    }
}

fn update_dynamics(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut RigidBody, With<Airplane>>,
) {
    let mut rigid_body = query.single_mut();

    let mut tx = 0.0;
    if keys.pressed(KeyCode::Space) {
        tx += THRUST
    }

    let mut delta_aileron = 0.0;
    if keys.pressed(KeyCode::KeyD) {
        delta_aileron += 1.0;
    } else if keys.pressed(KeyCode::KeyA) {
        delta_aileron -= 1.0;
    }

    let mut delta_rudder = 0.0;
    if keys.pressed(KeyCode::KeyE) {
        delta_rudder -= 1.0;
    } else if keys.pressed(KeyCode::KeyQ) {
        delta_rudder += 1.0;
    }

    let mut delta_elevator = 0.0;
    if keys.pressed(KeyCode::KeyW) {
        delta_elevator += 1.0;
    } else if keys.pressed(KeyCode::KeyS) {
        delta_elevator -= 1.0;
    }

    let v =
        (rigid_body.u * rigid_body.u + rigid_body.v * rigid_body.v + rigid_body.w * rigid_body.w)
            .sqrt();

    let alfa = rigid_body.w.atan2(rigid_body.u);
    let mut beta = 0.0;
    if v != 0.0 {
        beta = (rigid_body.v / v).asin();
    }

    let q = 1.0 / 2.0 * RO * v * v;

    let c_z = -C_Z_ALFA * alfa - C_Z_0;
    let c_x = K_CX * (c_z - C_Z_DM) * (c_z - C_Z_DM) + C_X_DM;
    let c_y = C_Y_BETA * beta;

    let fx = -q * S * c_x;
    let fy = q * S * c_y;
    let fz = q * S * c_z;

    let c_l = -C_L_BETA * beta
        + C_L_DELTA_A * delta_aileron
        + C_L_P * rigid_body.p
        + C_L_R * rigid_body.r;
    let c_m = C_M_ALFA * alfa - C_M_DELTA_E * delta_elevator + C_M_Q * rigid_body.q;
    let c_n =
        C_N_BETA * beta - C_N_DELTA_R * delta_rudder + C_N_P * rigid_body.p + C_N_R * rigid_body.r;

    let l = q * S * L * c_l;
    let m = q * S * C_MEAN * c_m;
    let n = q * S * L * c_n;

    let u_dot = tx / M - G * rigid_body.theta.sin() - rigid_body.q * rigid_body.w
        + rigid_body.r * rigid_body.v
        + fx / M;

    let v_dot = G * rigid_body.theta.cos() * rigid_body.phi.sin() - rigid_body.r * rigid_body.u
        + rigid_body.p * rigid_body.w
        + fy / M;

    let w_dot = G * rigid_body.theta.cos() * rigid_body.phi.cos() - rigid_body.p * rigid_body.v
        + rigid_body.q * rigid_body.u
        + fz / M;

    let inverse_rot = Quat::from_euler(
        EulerRot::XYZ,
        rigid_body.phi,
        rigid_body.theta,
        rigid_body.psi,
    );

    let rot = inverse_rot.inverse();

    let p_dot = (n * I_XZ// TODO
        + l * I_Z
        + rigid_body.q * rigid_body.r * (I_Z * I_Z - I_Y * I_Z + I_XZ * I_XZ)
        + rigid_body.p * rigid_body.q * (I_Y * I_XZ - I_X * I_XZ + I_Z * I_XZ))
        / (I_X * I_Z - I_XZ * I_XZ);
    let q_dot = (m
        - rigid_body.r * rigid_body.p * (I_X - I_Z)
        - I_XZ * (rigid_body.p * rigid_body.p - rigid_body.r * rigid_body.r))
        / I_Y;
    let r_dot = (n * I_X// TODO
        + l * I_XZ
        + rigid_body.q * rigid_body.r * (I_Y * I_XZ - I_Z * I_XZ - I_X * I_XZ)
        + rigid_body.p * rigid_body.q * (I_XZ * I_XZ - I_Y * I_X + I_X * I_X))
        / (I_X * I_Z - I_XZ * I_XZ);

    println!(
        "m {} {} {}",
        m,
        -rigid_body.r * rigid_body.q * (I_X - I_Z),
        -I_XZ * (rigid_body.p * rigid_body.p - rigid_body.r * rigid_body.r)
    );
    println!("p_dot {}", p_dot);
    println!("q_dot {}", q_dot);

    rigid_body.u += u_dot * time.delta_seconds();
    rigid_body.v += v_dot * time.delta_seconds();
    rigid_body.w += w_dot * time.delta_seconds();

    rigid_body.p += p_dot * time.delta_seconds();
    rigid_body.q += q_dot * time.delta_seconds();
    rigid_body.r += r_dot * time.delta_seconds();

    let body_v = Vec3::new(rigid_body.u, rigid_body.v, rigid_body.w);

    let earth_v = inverse_rot.mul_vec3(body_v);

    let x_dot = earth_v.x;
    let y_dot = earth_v.y;
    let z_dot = earth_v.z;
    let phi_dot = rigid_body.p
        + rigid_body.q * rigid_body.phi.sin() * rigid_body.theta.tan()
        + rigid_body.r * rigid_body.phi.cos() * rigid_body.theta.tan();
    let theta_dot = rigid_body.q * rigid_body.phi.cos() - rigid_body.r * rigid_body.phi.sin();
    let psi_dot = (rigid_body.q * rigid_body.phi.sin() + rigid_body.r * rigid_body.phi.cos())
        / rigid_body.theta.cos();

    rigid_body.x += x_dot * time.delta_seconds();
    rigid_body.y += y_dot * time.delta_seconds();
    rigid_body.z += z_dot * time.delta_seconds();
    rigid_body.phi += phi_dot * time.delta_seconds();
    rigid_body.theta += theta_dot * time.delta_seconds();
    rigid_body.psi += psi_dot * time.delta_seconds();

    if rigid_body.z >= 0.0 {
        rigid_body.z = 0.0;
        let new_body_v = rot.mul_vec3(Vec3::new(earth_v.x, earth_v.y, 0.0));

        rigid_body.u = new_body_v.x;
        rigid_body.v = new_body_v.y;
        rigid_body.w = new_body_v.z;
    }

    println!("alfa {}", alfa);
    println!("{:?}", rigid_body);
}

fn update_transform(
    mut query: Query<&mut Transform, With<Airplane>>,
    body_query: Query<&RigidBody, With<Airplane>>,
) {
    let body = body_query.single();
    let mut transform = query.single_mut();

    transform.translation.x = body.y;
    transform.translation.y = -body.z + 3.0;
    transform.translation.z = -body.x;

    *transform = transform.with_rotation(
        Quat::from_euler(EulerRot::YXZ, -body.psi, body.theta, -body.phi)
            * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2),
    )
}

fn update_camera(
    mut query: Query<&mut Transform, (With<Camera>, Without<Airplane>)>,
    transform_query: Query<&Transform, With<Airplane>>,
) {
    let transform = transform_query.single();
    let mut camera = query.single_mut();

    camera.translation = transform.translation
        + Vec3::new(
            transform.down().x * 10.0,
            transform.down().y * 10.0,
            transform.down().z * 10.0,
        )
        + Vec3::new(
            transform.right().x * 3.0,
            transform.right().y * 3.0,
            transform.right().z * 3.0,
        );
    *camera = camera.looking_at(transform.translation, transform.right());
}

fn draw_axes(mut gizmos: Gizmos, query: Query<&Transform, With<Airplane>>) {
    for &transform in &query {
        gizmos.axes(transform, 10.0)
    }
}
