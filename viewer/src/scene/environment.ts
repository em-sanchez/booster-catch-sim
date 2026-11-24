import * as THREE from 'three';
import { Sky } from 'three/examples/jsm/objects/Sky.js';

export interface Lighting {
  sun: THREE.DirectionalLight;
  sunDir: THREE.Vector3;
  hemi: THREE.HemisphereLight;
  sky: Sky;
}

/** Physically based sky, sun with shadows, image-based lighting from the sky, light haze. */
export function buildEnvironment(renderer: THREE.WebGLRenderer, scene: THREE.Scene): Lighting {
  const sky = new Sky();
  sky.scale.setScalar(45000);
  const u = sky.material.uniforms;
  u.turbidity.value = 4.5;
  u.rayleigh.value = 1.6;
  u.mieCoefficient.value = 0.0045;
  u.mieDirectionalG.value = 0.86;

  // afternoon sun
  const elevation = 22, azimuth = 38;   // behind the default cameras: front-lit scene
  const phi = THREE.MathUtils.degToRad(90 - elevation);
  const theta = THREE.MathUtils.degToRad(azimuth);
  const sunDir = new THREE.Vector3().setFromSphericalCoords(1, phi, theta);
  u.sunPosition.value.copy(sunDir);
  scene.add(sky);

  // Image-based lighting from a sun-free sky gradient: the Sky shader's sun disc is ~1000x
  // brighter than the rest of the scene and would turn every reflective surface into a glare.
  // The directional light below supplies the sun's own highlights.
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new THREE.Scene();
  const envSky = new THREE.Mesh(new THREE.SphereGeometry(500, 32, 16), new THREE.ShaderMaterial({
    side: THREE.BackSide, depthWrite: false,
    uniforms: {
      zenith: { value: new THREE.Color(0.55, 1.05, 2.1) },
      horizon: { value: new THREE.Color(2.1, 2.2, 2.3) },
      ground: { value: new THREE.Color(0.8, 0.7, 0.55) },
      sunDir: { value: sunDir },
    },
    vertexShader: 'varying vec3 vDir; void main(){ vDir = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
    fragmentShader: /* glsl */ `
      uniform vec3 zenith; uniform vec3 horizon; uniform vec3 ground; uniform vec3 sunDir;
      varying vec3 vDir;
      void main() {
        vec3 d = normalize(vDir);
        vec3 c = d.y > 0.0 ? mix(horizon, zenith, pow(d.y, 0.45)) : mix(horizon, ground, pow(-d.y, 0.35));
        c += vec3(1.0, 0.9, 0.75) * 0.6 * pow(max(dot(d, sunDir), 0.0), 8.0);   // soft sun glow, no disc
        gl_FragColor = vec4(c, 1.0);
      }`,
  }));
  envScene.add(envSky);
  scene.environment = pmrem.fromScene(envScene, 0.02).texture;
  scene.environmentIntensity = 1.0;

  const sun = new THREE.DirectionalLight(0xfff0dc, 9.0);
  sun.position.copy(sunDir).multiplyScalar(600);
  sun.target.position.set(0, 0, 0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const sc = sun.shadow.camera;
  sc.left = -160; sc.right = 160; sc.top = 200; sc.bottom = -120; sc.near = 50; sc.far = 1500;
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.6;
  scene.add(sun, sun.target);

  const hemi = new THREE.HemisphereLight(0xbcd6ff, 0x6b5a44, 0.2);
  scene.add(hemi);

  scene.fog = new THREE.FogExp2(0xc4c9cc, 0.000085);
  return { sun, sunDir, hemi, sky };
}
