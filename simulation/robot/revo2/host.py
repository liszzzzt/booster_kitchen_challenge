from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional
import sys
import traceback

from core.config import SimulationConfig
from core.extension_config import (
    EXTENSION_POINT_PHYSICS,
    ExtensionDeclaration,
    filter_extension_declarations,
    load_extension_declarations,
)
from core.physics_transport import PhysicsCommand, PhysicsCommandMessage
from physics.extensions.registry import get_physics_extension


InitializeCallback = Callable[[Any, Any], None]
ResetCallback = Callable[[], None]
StepCallback = Callable[[Any], None]
DisposeCallback = Callable[[], None]
# old_model, old_data, new_model, new_data
ModelRecompiledCallback = Callable[[Any, Any, Any, Any], None]
# model, data, body_id
BodyPoseChangedCallback = Callable[[Any, Any, int], None]
# model, data; fired only after an explicit scene artifact is durable.
SceneExportedCallback = Callable[[Any, Any], None]
CommandHandler = Callable[[PhysicsCommandMessage, Any, Any], None]
SimulationStateProvider = Callable[[], "PhysicsSimulationStateContribution"]
DirtyProvider = Callable[[], bool]


@dataclass
class PhysicsSimulationStateContribution:
    robot_modes: Dict[str, str] = field(default_factory=dict)
    robot_pose_specs: Dict[str, Dict[str, int]] = field(default_factory=dict)
    all_robot_names: Optional[List[str]] = None

    def merge(self, other: "PhysicsSimulationStateContribution") -> None:
        self.robot_modes.update(other.robot_modes)
        self.robot_pose_specs.update(other.robot_pose_specs)
        if other.all_robot_names is not None:
            self.all_robot_names = list(other.all_robot_names)

    def has_data(self) -> bool:
        return bool(
            self.robot_modes
            or self.robot_pose_specs
            or self.all_robot_names is not None
        )


@dataclass
class _PhysicsExtensionContribution:
    name: str
    required: bool = False
    initialize_callbacks: List[InitializeCallback] = field(default_factory=list)
    reset_callbacks: List[ResetCallback] = field(default_factory=list)
    before_step_callbacks: List[StepCallback] = field(default_factory=list)
    after_step_callbacks: List[StepCallback] = field(default_factory=list)
    post_step_callbacks: List[StepCallback] = field(default_factory=list)
    dispose_callbacks: List[DisposeCallback] = field(default_factory=list)
    model_recompiled_callbacks: List[ModelRecompiledCallback] = field(default_factory=list)
    body_pose_changed_callbacks: List[BodyPoseChangedCallback] = field(default_factory=list)
    scene_exported_callbacks: List[SceneExportedCallback] = field(default_factory=list)
    command_handlers: Dict[PhysicsCommand, CommandHandler] = field(default_factory=dict)
    simulation_state_providers: List[SimulationStateProvider] = field(default_factory=list)
    dirty_providers: List[DirtyProvider] = field(default_factory=list)
    active: bool = True


class PhysicsLifecycleApi:
    def __init__(self, contribution: _PhysicsExtensionContribution):
        self._contribution = contribution

    def on_initialize(self, callback: InitializeCallback) -> None:
        self._contribution.initialize_callbacks.append(callback)

    def on_reset(self, callback: ResetCallback) -> None:
        self._contribution.reset_callbacks.append(callback)

    def on_before_step(self, callback: StepCallback) -> None:
        self._contribution.before_step_callbacks.append(callback)

    def on_after_step(self, callback: StepCallback) -> None:
        self._contribution.after_step_callbacks.append(callback)

    def on_post_step(self, callback: StepCallback) -> None:
        self._contribution.post_step_callbacks.append(callback)

    def on_dispose(self, callback: DisposeCallback) -> None:
        self._contribution.dispose_callbacks.append(callback)

    def on_model_recompiled(self, callback: ModelRecompiledCallback) -> None:
        """Register a callback fired after the scene is recompiled.

        Receives ``(old_model, old_data, new_model, new_data)``. Extensions
        that cache model/data must rebind here (the old objects are frozen at
        the previous topology).
        """
        self._contribution.model_recompiled_callbacks.append(callback)

    def on_body_pose_changed(self, callback: BodyPoseChangedCallback) -> None:
        """Register a callback fired after a freejoint body pose is edited."""
        self._contribution.body_pose_changed_callbacks.append(callback)

    def on_scene_exported(self, callback: SceneExportedCallback) -> None:
        """Register a callback fired after an explicit scene save succeeds."""
        self._contribution.scene_exported_callbacks.append(callback)


class PhysicsCommandApi:
    def __init__(self, contribution: _PhysicsExtensionContribution):
        self._contribution = contribution

    def register(self, command: PhysicsCommand, handler: CommandHandler) -> None:
        if command in self._contribution.command_handlers:
            raise ValueError(
                f"Physics extension '{self._contribution.name}' registered duplicate command "
                f"'{command.value}'."
            )
        self._contribution.command_handlers[command] = handler


class PhysicsSimulationStateApi:
    def __init__(self, contribution: _PhysicsExtensionContribution):
        self._contribution = contribution

    def register_provider(self, provider: SimulationStateProvider) -> None:
        self._contribution.simulation_state_providers.append(provider)

    def register_dirty_provider(self, provider: DirtyProvider) -> None:
        self._contribution.dirty_providers.append(provider)


class PhysicsExtensionContext:
    def __init__(
        self,
        contribution: _PhysicsExtensionContribution,
        *,
        model_path: str,
        simulation_config: SimulationConfig,
    ):
        self.lifecycle = PhysicsLifecycleApi(contribution)
        self.commands = PhysicsCommandApi(contribution)
        self.simulation_state = PhysicsSimulationStateApi(contribution)
        self.model_path = model_path
        self.simulation_config = simulation_config


class PhysicsExtensionHost:
    def __init__(self, *, model_path: str, simulation_config: SimulationConfig):
        self._model_path = model_path
        self._simulation_config = simulation_config
        self._contributions: List[_PhysicsExtensionContribution] = []

    def activate(self, declarations: Iterable[ExtensionDeclaration]) -> None:
        from physics.extensions.builtin import register_builtin_physics_extensions
        from physics.extensions.robot_controller.builtin import install_builtin_robot_controller

        register_builtin_physics_extensions()

        builtin_contribution = _PhysicsExtensionContribution(
            name="robot_controller",
            required=True,
        )
        builtin_context = PhysicsExtensionContext(
            builtin_contribution,
            model_path=self._model_path,
            simulation_config=self._simulation_config,
        )
        try:
            install_builtin_robot_controller(builtin_context)
            self._commit_contribution(builtin_contribution)
        except Exception as exc:
            self._dispose_contribution(builtin_contribution)
            raise RuntimeError(
                f"Failed to install built-in robot controller: {exc}"
            ) from exc

        for declaration in declarations:
            if declaration.name == "robot_controller":
                self._fatal(
                    "'robot_controller' is a built-in physics capability and must not "
                    "be declared in scene extension metadata."
                )
            definition = get_physics_extension(declaration.name)
            if definition is None:
                self._fatal(
                    f"Physics extension '{declaration.name}' is not registered. "
                    "Please check your .extensions.xml file."
                )

            contribution = _PhysicsExtensionContribution(name=declaration.name)
            context = PhysicsExtensionContext(
                contribution,
                model_path=self._model_path,
                simulation_config=self._simulation_config,
            )
            try:
                config = definition.parse_config(dict(declaration.params))
                definition.contribute(context, config)
                self._commit_contribution(contribution)
            except Exception as exc:
                self._dispose_contribution(contribution)
                self._fatal(f"Failed to activate physics extension '{declaration.name}': {exc}")

        # --- robo@home revo2_hand: always on (idle without Revo2 actuators) ---
        if not any(c.name == 'revo2_hand' for c in self._contributions):
            _revo2 = get_physics_extension('revo2_hand')
            if _revo2 is not None:
                _c = _PhysicsExtensionContribution(name='revo2_hand')
                try:
                    _revo2.contribute(PhysicsExtensionContext(_c, model_path=self._model_path,
                                                              simulation_config=self._simulation_config), _revo2.parse_config({}))
                    self._commit_contribution(_c)
                except Exception as exc:
                    self._dispose_contribution(_c)
                    print(f'[PhysicsExtensionHost] revo2_hand auto-activation failed: {exc}')
        # --- end revo2_hand ---

    def initialize(self, model: Any, data: Any) -> None:
        for contribution in list(self._active_contributions()):
            try:
                for callback in contribution.initialize_callbacks:
                    callback(model, data)
            except Exception as exc:
                print(
                    f"[PhysicsExtensionHost] Physics extension '{contribution.name}' "
                    f"initialization failed: {exc}"
                )
                traceback.print_exc()
                self._deactivate_contribution(contribution)
                if contribution.required:
                    raise RuntimeError(
                        f"Required physics capability '{contribution.name}' failed to initialize"
                    ) from exc

    def on_reset(self) -> None:
        for contribution in list(self._active_contributions()):
            for callback in contribution.reset_callbacks:
                try:
                    callback()
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] Reset hook error in '{contribution.name}': {exc}")
                    traceback.print_exc()

    def before_step(self, profiler: Any) -> None:
        for contribution in list(self._active_contributions()):
            for callback in contribution.before_step_callbacks:
                try:
                    callback(profiler)
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] Before-step hook error in '{contribution.name}': {exc}")
                    traceback.print_exc()

    def after_step(self, profiler: Any) -> None:
        for contribution in list(self._active_contributions()):
            for callback in contribution.after_step_callbacks:
                try:
                    callback(profiler)
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] After-step hook error in '{contribution.name}': {exc}")
                    traceback.print_exc()

    def post_step(self, profiler: Any) -> None:
        for contribution in list(self._active_contributions()):
            for callback in contribution.post_step_callbacks:
                try:
                    callback(profiler)
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] Post-step hook error in '{contribution.name}': {exc}")
                    traceback.print_exc()

    def on_model_recompiled(self, old_model: Any, old_data: Any, new_model: Any, new_data: Any) -> None:
        """Notify every active extension that the scene was recompiled."""
        for contribution in list(self._active_contributions()):
            for callback in contribution.model_recompiled_callbacks:
                try:
                    callback(old_model, old_data, new_model, new_data)
                except Exception as exc:
                    print(
                        f"[PhysicsExtensionHost] Model-recompiled hook error in "
                        f"'{contribution.name}': {exc}"
                    )
                    traceback.print_exc()

    def on_body_pose_changed(self, model: Any, data: Any, body_id: int) -> None:
        """Notify active extensions that a freejoint body pose was edited."""
        for contribution in list(self._active_contributions()):
            for callback in contribution.body_pose_changed_callbacks:
                try:
                    callback(model, data, body_id)
                except Exception as exc:
                    print(
                        f"[PhysicsExtensionHost] Body-pose-changed hook error in "
                        f"'{contribution.name}': {exc}"
                    )
                    traceback.print_exc()

    def on_scene_exported(self, model: Any, data: Any) -> None:
        """Notify extensions that the current scene snapshot was saved."""
        for contribution in list(self._active_contributions()):
            for callback in contribution.scene_exported_callbacks:
                try:
                    callback(model, data)
                except Exception as exc:
                    print(
                        f"[PhysicsExtensionHost] Scene-exported hook error in "
                        f"'{contribution.name}': {exc}"
                    )
                    traceback.print_exc()

    def handle_command(self, cmd: PhysicsCommandMessage, model: Any, data: Any) -> bool:
        for contribution in self._active_contributions():
            handler = contribution.command_handlers.get(cmd.command)
            if handler is None:
                continue
            handler(cmd, model, data)
            return True
        return False

    def consume_simulation_state_dirty(self) -> bool:
        dirty = False
        for contribution in list(self._active_contributions()):
            for provider in contribution.dirty_providers:
                try:
                    dirty = bool(provider()) or dirty
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] Dirty provider error in '{contribution.name}': {exc}")
                    traceback.print_exc()
        return dirty

    def get_simulation_state_contribution(self) -> PhysicsSimulationStateContribution:
        state = PhysicsSimulationStateContribution()
        for contribution in list(self._active_contributions()):
            for provider in contribution.simulation_state_providers:
                try:
                    state.merge(provider())
                except Exception as exc:
                    print(f"[PhysicsExtensionHost] State provider error in '{contribution.name}': {exc}")
                    traceback.print_exc()
        return state

    def dispose(self) -> None:
        for contribution in reversed(self._contributions):
            self._dispose_contribution(contribution)

    def _commit_contribution(self, contribution: _PhysicsExtensionContribution) -> None:
        for existing in self._contributions:
            for command in contribution.command_handlers:
                if command in existing.command_handlers:
                    self._fatal(
                        f"Physics command '{command.value}' is already registered by "
                        f"extension '{existing.name}'."
                    )
        self._contributions.append(contribution)

    def _active_contributions(self) -> Iterable[_PhysicsExtensionContribution]:
        return (contribution for contribution in self._contributions if contribution.active)

    def _deactivate_contribution(self, contribution: _PhysicsExtensionContribution) -> None:
        if not contribution.active:
            return
        contribution.active = False
        self._dispose_contribution(contribution)

    def _dispose_contribution(self, contribution: _PhysicsExtensionContribution) -> None:
        for callback in reversed(contribution.dispose_callbacks):
            try:
                callback()
            except Exception as exc:
                print(f"[PhysicsExtensionHost] Dispose hook error in '{contribution.name}': {exc}")
                traceback.print_exc()

    def _fatal(self, message: str) -> None:
        print(
            f"\n{'=' * 80}\n"
            f"FATAL ERROR: Invalid physics extension configuration\n"
            f"{'=' * 80}\n"
            f"{message}\n"
            f"{'=' * 80}\n",
            file=sys.stderr,
        )
        sys.exit(1)


def load_physics_extension_host(
    model_path: str,
    simulation_config: SimulationConfig,
    declarations: Optional[Iterable[ExtensionDeclaration]] = None,
) -> PhysicsExtensionHost:
    declarations = filter_extension_declarations(
        load_extension_declarations(model_path) if declarations is None else declarations,
        EXTENSION_POINT_PHYSICS,
    )
    host = PhysicsExtensionHost(
        model_path=model_path,
        simulation_config=simulation_config,
    )
    host.activate(declarations)
    return host
