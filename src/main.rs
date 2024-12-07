//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::prelude::*;

const THRUST: f32 = 1.0;
const MG: f32 = 0.2;
const CD0: f32 = 0.04;
const DCD: f32 = 0.1;
const CL0: f32 = 0.0;
const DCL: f32 = 1.0;
const DCMP: f32 = 1.0;
const DELTA: f32 = 0.02;
const G: f32 = 9.81;
const M: f32 = 1000.0;
const I_X: f32 = 1.0;
const I_Y: f32 = 1.0;
const I_Z: f32 = 1.0;
const I_XZ: f32 = 1.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, (setup_airplane, setup_world))
        .add_systems(
            Update,
            (update_dynamics, update_transform, update_camera, draw_axes).chain(),
        )
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
) {
    commands.spawn(PbrBundle {
        mesh: meshes.add(Circle::new(100.0)),
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

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-60.0, 60.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        Camera,
    ));
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

    let mut delta = 0.0;
    if keys.pressed(KeyCode::KeyW) {
        delta += DELTA;
    } else if keys.pressed(KeyCode::KeyS) {
        delta -= DELTA;
    }

    // aerodynamic forces TODO
    let fx = 0.0;
    let fy = 0.0;
    let fz = 0.0;

    // aerodynamic moments TODO
    let l = 0.0;
    let m = 0.0;
    let n = 0.0;

    let u_dot = tx / M - G * rigid_body.theta.sin() - rigid_body.q * rigid_body.w
        + rigid_body.r * rigid_body.v
        + fx / M;

    let v_dot = G * rigid_body.theta.cos() * rigid_body.phi.sin() - rigid_body.r * rigid_body.u
        + rigid_body.p * rigid_body.w
        + fy / M;

    let w_dot = G * rigid_body.theta.cos() * rigid_body.phi.cos() - rigid_body.p * rigid_body.v
        + rigid_body.q * rigid_body.u
        + fz / M;

    let p_dot = (n * I_XZ
        + l * I_Z
        + rigid_body.q * rigid_body.r * (I_Z * I_Z - I_Y * I_Z + I_XZ * I_XZ)
        + rigid_body.p * rigid_body.q * (I_Y * I_XZ - I_X * I_XZ + I_Z * I_XZ))
        / (I_X * I_Z - I_XZ * I_XZ);
    let q_dot = (m
        - rigid_body.r * rigid_body.q * (I_X - I_Z)
        - I_XZ * (rigid_body.p * rigid_body.p - rigid_body.r * rigid_body.r))
        / I_Y;
    let r_dot = (n * I_X
        + l * I_XZ
        + rigid_body.q * rigid_body.r * (I_Y * I_XZ - I_Z * I_XZ - I_X * I_XZ)
        + rigid_body.p * rigid_body.q * (I_XZ * I_XZ - I_Y * I_X + I_X * I_X))
        / (I_X * I_Z - I_XZ * I_XZ);

    rigid_body.u += u_dot * time.delta_seconds();
    rigid_body.v += v_dot * time.delta_seconds();
    rigid_body.w += w_dot * time.delta_seconds();
    rigid_body.p += p_dot * time.delta_seconds();
    rigid_body.q += q_dot * time.delta_seconds();
    rigid_body.r += r_dot * time.delta_seconds();

    let v = Vec3::new(rigid_body.u, rigid_body.v, rigid_body.w);
    let rot = Quat::from_euler(
        EulerRot::YXZ,
        rigid_body.phi.to_radians(),
        rigid_body.theta.to_radians(),
        rigid_body.psi.to_radians(),
    );

    let rotated_v = rot.mul_vec3(v);

    let x_dot = rotated_v.x;
    let y_dot = rotated_v.y;
    let z_dot = rotated_v.z;
    let phi_dot = rigid_body.q * rigid_body.phi.cos() - rigid_body.r * rigid_body.phi.sin();
    let theta_dot = rigid_body.p
        + rigid_body.q * rigid_body.phi.sin() * rigid_body.theta.tan()
        + rigid_body.r * rigid_body.phi.cos() * rigid_body.theta.tan();
    let psi_dot = (rigid_body.q * rigid_body.phi.sin() + rigid_body.r * rigid_body.phi.cos())
        / rigid_body.theta.cos();

    rigid_body.x += x_dot * time.delta_seconds();
    rigid_body.y += y_dot * time.delta_seconds();
    rigid_body.z += z_dot * time.delta_seconds();
    rigid_body.phi += phi_dot * time.delta_seconds();
    rigid_body.theta += theta_dot * time.delta_seconds();
    rigid_body.psi += psi_dot * time.delta_seconds();
}

fn update_transform(
    mut query: Query<&mut Transform, With<Airplane>>,
    body_query: Query<&RigidBody, With<Airplane>>,
) {
    let body = body_query.single();
    let mut transform = query.single_mut();

    // TODO
    transform.translation.x = body.x;
    transform.translation.y = body.y;

    println!("{}", transform.rotation);

    *transform = transform.with_rotation(
        Quat::from_rotation_z(body.p) * Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2),
    )
}

fn update_camera(
    mut query: Query<&mut Transform, (With<Camera>, Without<Airplane>)>,
    transform_query: Query<&Transform, With<Airplane>>,
) {
    let transform = transform_query.single();
    let mut camera = query.single_mut();

    // TODO
    camera.translation = transform.translation
        - Vec3::new(
            transform.up().x * 10.0,
            transform.up().y * 10.0,
            transform.up().z * 10.0,
        )
        + Vec3::new(
            transform.left().x * 3.0,
            transform.left().y * 3.0,
            transform.left().z * 3.0,
        );
    *camera = camera.looking_at(transform.translation, transform.left());
}

fn draw_axes(mut gizmos: Gizmos, query: Query<&Transform, With<Airplane>>) {
    for &transform in &query {
        gizmos.axes(transform, 10.0)
    }
}
