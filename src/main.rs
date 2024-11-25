//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::prelude::*;

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
#[derive(Component)]
struct RigidBody {
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    // pub r: f32,
    // pub p: f32,
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
                vx: 10.0,
                vy: 0.1,
                vz: 0.0,
                // r: 0.0,
                // p: 0.0,
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

fn update_dynamics(time: Res<Time>, mut query: Query<&mut RigidBody, With<Airplane>>) {
    for mut rigid_body in &mut query {
        let current_vx = rigid_body.vx;
        let current_vz = rigid_body.vz;
        rigid_body.vx += -current_vz * time.delta_seconds();
        rigid_body.vz += current_vx * time.delta_seconds();
    }
}

fn update_transform(
    time: Res<Time>,
    mut query: Query<&mut Transform, With<Airplane>>,
    body_query: Query<&RigidBody, With<Airplane>>,
) {
    let body = body_query.single();

    for mut transform in &mut query {
        transform.translation.x += body.vx * time.delta_seconds();
        transform.translation.y += body.vy * time.delta_seconds();
        transform.translation.z += body.vz * time.delta_seconds();

        transform.rotate_y(-time.delta_seconds() / 1.5 * std::f32::consts::FRAC_PI_2);
        transform.rotate_local_y(-time.delta_seconds() / 1.5 * std::f32::consts::FRAC_PI_2);
        transform.rotate_local_z(
            (time.elapsed_seconds() * std::f32::consts::FRAC_PI_2).cos() * time.delta_seconds(),
        );
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
