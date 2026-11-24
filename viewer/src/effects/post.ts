import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';

export class Post {
  readonly composer: EffectComposer;
  readonly bloom: UnrealBloomPass;
  private render: RenderPass;

  constructor(renderer: THREE.WebGLRenderer, scene: THREE.Scene, camera: THREE.Camera) {
    const size = renderer.getSize(new THREE.Vector2());
    const target = new THREE.WebGLRenderTarget(size.x, size.y, { type: THREE.HalfFloatType, samples: 4 });
    this.composer = new EffectComposer(renderer, target);
    this.render = new RenderPass(scene, camera);
    this.composer.addPass(this.render);
    this.bloom = new UnrealBloomPass(size, 0.5, 0.35, 5.0);
    this.composer.addPass(this.bloom);
    this.composer.addPass(new OutputPass());
  }

  setCamera(camera: THREE.Camera): void { this.render.camera = camera; }

  setSize(w: number, h: number): void {
    this.composer.setSize(w, h);
    this.bloom.resolution.set(w, h);
  }
}
