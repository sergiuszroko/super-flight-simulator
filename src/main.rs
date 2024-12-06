//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::prelude::*;

const THRUST: f64 = 1.0;
const MG: f64 = 0.2;
const CD0: f64 = 0.04;
const DCD: f64 = 0.1;
const CL0: f64 = 0.0;
const DCL: f64 = 1.0;
const DCMP: f64 = 1.0;
const DELTA: f64 = 0.02;

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
    pub vx: f64,
    pub vy: f64,
    pub x: f64,
    pub y: f64,
    // pub vz: f64,
    pub omega_p: f64,
    // pub r: f32,
    pub p: f64,
    // pub y: f32,
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
                vx: 0.0,
                vy: 0.0,
                x: 0.0,
                y: 0.0,
                // vz: 0.0,
                omega_p: 0.0,
                // r: 0.0,
                p: 0.0,
                // y: 0.0,
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
    for mut rigid_body in &mut query {
        let mut t = 0.0;
        if keys.pressed(KeyCode::Space) {
            t += THRUST
        }

        let mut d = 0.0;
        if keys.pressed(KeyCode::KeyW) {
            d += DELTA;
        } else if keys.pressed(KeyCode::KeyS) {
            d -= DELTA;
        }

        let v2 = rigid_body.vx * rigid_body.vx + rigid_body.vy + rigid_body.vy;

        let mut lift = 0.0;
        let mut drag = 0.0;
        let mut m_p = 0.0;
        let mut alfa = 0.0;

        if v2 > 0.0 {
            let gamma = -(rigid_body.vy / v2.sqrt()).asin();
            alfa = rigid_body.p + gamma;

            lift = v2 / 2.0 * (CL0 + DCL * alfa);
            drag = v2 / 2.0 * (CD0 + DCD * alfa);
            m_p = -v2 / 2.0 * (DCMP * alfa + d);
        }

        let ax = t * alfa.cos() - drag * alfa.cos() + lift * alfa.sin();
        let ay = t * alfa.sin() + drag * alfa.sin() + lift * alfa.cos() - MG;

        let eps = m_p;

        rigid_body.vx += ax * time.delta_seconds_f64();
        rigid_body.vy += ay * time.delta_seconds_f64();

        rigid_body.x += rigid_body.vx * time.delta_seconds_f64();
        rigid_body.y += rigid_body.vy * time.delta_seconds_f64();

        rigid_body.omega_p += eps * time.delta_seconds_f64();

        rigid_body.p += rigid_body.omega_p * time.delta_seconds_f64();

        if rigid_body.y < 5.0 {
            rigid_body.y = 5.0;
            rigid_body.vy = 0.0;
        }

        // println!(
        //     "lift {}, drag {}, m_p {}, ax {}, ay {}, rigid_body {:?}",
        //     lift, drag, m_p, ax, ay, rigid_body
        // );

        println!("{:?}", rigid_body);
    }
}

fn update_transform(
    mut query: Query<&mut Transform, With<Airplane>>,
    body_query: Query<&RigidBody, With<Airplane>>,
) {
    let body = body_query.single();

    for mut transform in &mut query {
        transform.translation.x = body.x as f32;
        transform.translation.y = body.y as f32;

        println!("{}", transform.rotation);

        *transform = transform.with_rotation(
            Quat::from_rotation_z(body.p as f32)
                * Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2),
        )
    }
}

fn update_camera(
    mut query: Query<&mut Transform, (With<Camera>, Without<Airplane>)>,
    transform_query: Query<&Transform, With<Airplane>>,
) {
    let transform = transform_query.single();

    for mut camera in &mut query {
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
}

fn draw_axes(mut gizmos: Gizmos, query: Query<&Transform, With<Airplane>>) {
    for &transform in &query {
        gizmos.axes(transform, 10.0)
    }
}
